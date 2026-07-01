from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Any

from tcr_minibot.odometry.encoder_interface import EncoderReader

try:  # pragma: no cover - exercised on the Raspberry Pi
    from gpiozero import DigitalInputDevice  # type: ignore
except Exception:  # pragma: no cover - lets non-Pi dev machines import this module
    DigitalInputDevice = None  # type: ignore[assignment]


@dataclass(frozen=True)
class QuadratureEncoderPins:
    """BCM GPIO pins for one two-channel quadrature encoder."""

    channel_a_gpio: int
    channel_b_gpio: int
    name: str = "encoder"
    pull_up: bool = True
    invert: bool = False


@dataclass(frozen=True)
class DriveEncoderPins:
    """
    Two-wheel encoder pin map.

    Defaults match the swapped wiring reported after the first encoder test:
    LEFT encoder is on GPIO27/GPIO17 and RIGHT encoder is on GPIO4/GPIO22.
    Override these from the CLI if you move wires around later.
    """

    left: QuadratureEncoderPins = QuadratureEncoderPins(27, 17, name="left")
    right: QuadratureEncoderPins = QuadratureEncoderPins(4, 22, name="right")


@dataclass(frozen=True)
class EncoderSnapshot:
    name: str
    count: int
    raw_count: int
    state: int
    edge_count: int
    bad_transition_count: int


class QuadratureEncoder:
    """
    Small gpiozero-based quadrature decoder.

    The count increases or decreases by one for every valid A/B edge transition.
    That means the ticks-per-wheel-revolution value you use for odometry should
    be measured with this decoder, not copied blindly from a motor product page.
    """

    _TRANSITION_TABLE = (
        0,
        1,
        -1,
        0,
        -1,
        0,
        0,
        1,
        1,
        0,
        0,
        -1,
        0,
        -1,
        1,
        0,
    )

    def __init__(self, pins: QuadratureEncoderPins) -> None:
        if DigitalInputDevice is None:
            raise RuntimeError(
                "gpiozero is not installed/importable. On the Pi run: "
                "sudo apt install -y python3-gpiozero python3-lgpio"
            )

        self.pins = pins
        self._lock = Lock()
        self._raw_count = 0
        self._edge_count = 0
        self._bad_transition_count = 0

        self._a = DigitalInputDevice(pins.channel_a_gpio, pull_up=pins.pull_up)
        self._b = DigitalInputDevice(pins.channel_b_gpio, pull_up=pins.pull_up)
        self._last_state = self._read_state_unlocked()

        self._a.when_activated = self._on_edge
        self._a.when_deactivated = self._on_edge
        self._b.when_activated = self._on_edge
        self._b.when_deactivated = self._on_edge

    def _read_state_unlocked(self) -> int:
        return (int(self._a.value) << 1) | int(self._b.value)

    def _on_edge(self, *args: Any, **kwargs: Any) -> None:
        del args, kwargs
        with self._lock:
            new_state = self._read_state_unlocked()
            transition = (self._last_state << 2) | new_state
            delta = self._TRANSITION_TABLE[transition]
            if delta == 0 and new_state != self._last_state:
                self._bad_transition_count += 1
            self._raw_count += delta
            self._edge_count += 1
            self._last_state = new_state

    @property
    def count(self) -> int:
        with self._lock:
            return -self._raw_count if self.pins.invert else self._raw_count

    def reset(self) -> None:
        with self._lock:
            self._raw_count = 0
            self._edge_count = 0
            self._bad_transition_count = 0
            self._last_state = self._read_state_unlocked()

    def snapshot(self) -> EncoderSnapshot:
        with self._lock:
            raw = self._raw_count
            return EncoderSnapshot(
                name=self.pins.name,
                count=-raw if self.pins.invert else raw,
                raw_count=raw,
                state=self._last_state,
                edge_count=self._edge_count,
                bad_transition_count=self._bad_transition_count,
            )

    def close(self) -> None:
        self._a.close()
        self._b.close()


class GpioZeroEncoderReader(EncoderReader):
    """EncoderReader implementation for two quadrature wheel encoders."""

    def __init__(self, pins: DriveEncoderPins = DriveEncoderPins()) -> None:
        self.pins = pins
        self.left = QuadratureEncoder(pins.left)
        self.right = QuadratureEncoder(pins.right)

    def read_left_ticks(self) -> int:
        return self.left.count

    def read_right_ticks(self) -> int:
        return self.right.count

    def reset(self) -> None:
        self.left.reset()
        self.right.reset()

    def snapshot(self) -> tuple[EncoderSnapshot, EncoderSnapshot]:
        return self.left.snapshot(), self.right.snapshot()

    def close(self) -> None:
        self.left.close()
        self.right.close()
