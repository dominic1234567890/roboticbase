from __future__ import annotations

from dataclasses import dataclass
import json
import math
from time import monotonic, sleep
from typing import Any
from urllib.parse import quote
from urllib.request import urlopen


BUFFER_ALIASES: dict[str, tuple[str, ...]] = {
    "x": ("x", "gyrX", "gyroX"),
    "y": ("y", "gyrY", "gyroY"),
    "z": ("z", "gyrZ", "gyroZ"),
    "t": ("t", "gyr_time", "time"),
}


@dataclass(frozen=True)
class PhyphoxConfig:
    """Connection settings for phyphox Remote Access.

    iPhones usually show a URL like http://192.168.1.42 when Remote Access is
    enabled. Android often includes :8080. Pass the exact URL phyphox shows.
    """

    base_url: str
    yaw_rate_buffer: str = "z"
    timeout_s: float = 0.25
    yaw_rate_scale: float = 1.0
    invert_yaw_rate: bool = False


class PhyphoxError(RuntimeError):
    pass


class PhyphoxPhoneGyro:
    """Small phyphox REST client for using a phone gyro as a temporary yaw sensor.

    phyphox Gyroscope buffer names vary by platform/experiment. Some expose
    x/y/z, while the iPhone Gyroscope (rotation rate) experiment exposes gyrX,
    gyrY, and gyrZ. You can pass either style; simple axis names are resolved
    automatically from the experiment config when possible.
    """

    def __init__(self, config: PhyphoxConfig) -> None:
        if not config.base_url.strip():
            raise ValueError("Phyphox base_url is required")
        self.config = config
        self.base_url = normalize_base_url(config.base_url)
        self.bias_radps = 0.0
        self.last_raw_radps: float | None = None
        self.last_yaw_rate_radps: float | None = None
        self.last_sample_s: float | None = None
        self.last_error: str | None = None
        self._resolved_yaw_rate_buffer: str | None = None

    def fetch_json(self, path_and_query: str) -> dict[str, Any]:
        url = f"{self.base_url}/{path_and_query.lstrip('/')}"
        try:
            with urlopen(url, timeout=max(0.05, self.config.timeout_s)) as response:
                payload = response.read().decode("utf-8")
        except Exception as exc:  # noqa: BLE001 - caller needs the original message
            self.last_error = f"{type(exc).__name__}: {exc}"
            raise PhyphoxError(f"Could not reach phyphox at {url}: {exc}") from exc

        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            self.last_error = f"JSONDecodeError: {exc}"
            raise PhyphoxError(f"phyphox returned non-JSON data from {url}") from exc
        return data

    def get_config(self) -> dict[str, Any]:
        return self.fetch_json("config")

    def get_meta(self) -> dict[str, Any]:
        return self.fetch_json("meta")

    def control(self, command: str) -> bool:
        data = self.fetch_json(f"control?cmd={quote(command)}")
        return bool(data.get("result"))

    def start(self) -> bool:
        return self.control("start")

    def stop(self) -> bool:
        return self.control("stop")

    def clear(self) -> bool:
        return self.control("clear")

    def buffer_names(self) -> list[str]:
        try:
            cfg = self.get_config()
        except PhyphoxError:
            return []
        return buffer_names_from_config(cfg)

    def resolve_yaw_rate_buffer(self) -> str:
        if self._resolved_yaw_rate_buffer is not None:
            return self._resolved_yaw_rate_buffer

        requested = self.config.yaw_rate_buffer.strip()
        if not requested:
            raise PhyphoxError("Empty yaw-rate buffer name")

        available = self.buffer_names()
        available_set = set(available)
        if requested in available_set:
            self._resolved_yaw_rate_buffer = requested
            return requested

        for alias in BUFFER_ALIASES.get(requested, (requested,)):
            if alias in available_set:
                print(f"Resolved phyphox buffer {requested!r} -> {alias!r}")
                self._resolved_yaw_rate_buffer = alias
                return alias

        # If config lookup failed or did not list buffers, keep the user-provided
        # value so the /get error can show what phyphox actually returned.
        self._resolved_yaw_rate_buffer = requested
        return requested

    def read_yaw_rate_radps(self) -> float:
        raw = self._read_raw_yaw_rate_radps()
        sign = -1.0 if self.config.invert_yaw_rate else 1.0
        yaw_rate = sign * (raw - self.bias_radps) * self.config.yaw_rate_scale
        self.last_raw_radps = raw
        self.last_yaw_rate_radps = yaw_rate
        self.last_sample_s = monotonic()
        self.last_error = None
        return yaw_rate

    def calibrate_bias(self, *, duration_s: float = 2.0, sample_hz: float = 30.0) -> float:
        """Average the yaw-rate buffer while the robot/phone is completely still."""

        values: list[float] = []
        end_s = monotonic() + max(0.1, duration_s)
        period_s = 1.0 / max(1.0, sample_hz)
        while monotonic() < end_s:
            try:
                values.append(self._read_raw_yaw_rate_radps())
            except PhyphoxError:
                # Wi-Fi can hiccup. Ignore isolated misses during calibration.
                pass
            sleep(period_s)

        if not values:
            names = self.buffer_names()
            available = ", ".join(names) if names else "unknown"
            raise PhyphoxError(
                "Could not collect any phone gyro samples for calibration. "
                f"Requested buffer={self.config.yaw_rate_buffer!r}; available buffers={available}"
            )
        self.bias_radps = sum(values) / len(values)
        return self.bias_radps

    def _read_raw_yaw_rate_radps(self) -> float:
        buffer_name = self.resolve_yaw_rate_buffer()
        data = self.fetch_json(f"get?{quote(buffer_name)}")
        buffer_root = data.get("buffer")
        if not isinstance(buffer_root, dict):
            raise PhyphoxError("phyphox /get response did not include a buffer object")
        entry = buffer_root.get(buffer_name)
        if not isinstance(entry, dict):
            available = ", ".join(sorted(buffer_root.keys()))
            raise PhyphoxError(
                f"Buffer {buffer_name!r} was not returned by phyphox. "
                f"Returned buffers: {available or '(none)'}"
            )
        values = entry.get("buffer")
        if not isinstance(values, list) or not values:
            raise PhyphoxError(f"Buffer {buffer_name!r} was empty")

        for value in reversed(values):
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                return float(value)
        raise PhyphoxError(f"Buffer {buffer_name!r} had no finite numeric values")


def buffer_names_from_config(cfg: dict[str, Any]) -> list[str]:
    buffers = cfg.get("buffers", [])
    names: list[str] = []
    if isinstance(buffers, list):
        for item in buffers:
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                names.append(item["name"])
    return names


def normalize_base_url(raw_url: str) -> str:
    value = raw_url.strip().rstrip("/")
    if not value:
        raise ValueError("Empty phyphox URL")
    if "://" not in value:
        value = f"http://{value}"
    return value
