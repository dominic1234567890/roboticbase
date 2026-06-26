from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Iterator

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover - lets tests run without hardware deps
    serial = None

from tcr_minibot.utils.geometry import ld20_clockwise_to_robot_ccw, polar_to_xy, wrap_deg

POINTS_PER_PACKET = 12
HEADER = 0x54
VER_LEN = 0x2C
FRAME_LEN = 47

def crc8(data: bytes) -> int:
    """LD-series CRC-8: polynomial 0x4D, init 0x00, no reflection, xorout 0x00."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x4D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc



@dataclass(frozen=True)
class LidarPoint:
    raw_angle_deg_cw: float
    bearing_deg: float
    distance_m: float
    confidence: int
    x_m: float
    y_m: float
    timestamp_ms: int


@dataclass(frozen=True)
class LidarFrame:
    speed_deg_per_s: float
    start_angle_deg_cw: float
    end_angle_deg_cw: float
    timestamp_ms: int
    points: list[LidarPoint]
    crc_ok: bool



def _u16le(data: bytes, idx: int) -> int:
    return data[idx] | (data[idx + 1] << 8)


class LD20Parser:
    def __init__(self, mount_yaw_offset_deg: float = 0.0, check_crc: bool = True) -> None:
        self.buffer = bytearray()
        self.mount_yaw_offset_deg = mount_yaw_offset_deg
        self.check_crc = check_crc
        self.bad_crc_count = 0
        self.frame_count = 0

    def feed(self, data: bytes) -> list[LidarFrame]:
        self.buffer.extend(data)
        frames: list[LidarFrame] = []

        while True:
            header_idx = self.buffer.find(bytes([HEADER, VER_LEN]))
            if header_idx < 0:
                # Keep last byte in case it is the first half of a split header.
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if header_idx > 0:
                del self.buffer[:header_idx]
            if len(self.buffer) < FRAME_LEN:
                break

            candidate = bytes(self.buffer[:FRAME_LEN])
            del self.buffer[:FRAME_LEN]

            frame = self.parse_frame(candidate)
            if frame is not None:
                frames.append(frame)
        return frames

    def parse_frame(self, frame: bytes) -> LidarFrame | None:
        if len(frame) != FRAME_LEN:
            raise ValueError(f"LD20 frame must be {FRAME_LEN} bytes, got {len(frame)}")
        if frame[0] != HEADER or frame[1] != VER_LEN:
            return None

        crc_ok = crc8(frame[:-1]) == frame[-1]
        if self.check_crc and not crc_ok:
            self.bad_crc_count += 1
            return None

        speed_raw = _u16le(frame, 2)
        start_raw = _u16le(frame, 4)
        end_raw = _u16le(frame, 42)
        timestamp_ms = _u16le(frame, 44)

        start_deg = start_raw / 100.0
        end_deg = end_raw / 100.0
        # Clockwise wrap-aware interpolation in LD20's raw angle convention.
        span = (end_deg - start_deg) % 360.0
        step = span / (POINTS_PER_PACKET - 1)

        points: list[LidarPoint] = []
        idx = 6
        for i in range(POINTS_PER_PACKET):
            distance_mm = _u16le(frame, idx)
            confidence = frame[idx + 2]
            raw_angle = wrap_deg(start_deg + step * i)
            bearing = ld20_clockwise_to_robot_ccw(raw_angle, self.mount_yaw_offset_deg)
            distance_m = distance_mm / 1000.0
            x_m, y_m = polar_to_xy(distance_m, bearing)
            points.append(
                LidarPoint(
                    raw_angle_deg_cw=raw_angle,
                    bearing_deg=bearing,
                    distance_m=distance_m,
                    confidence=confidence,
                    x_m=x_m,
                    y_m=y_m,
                    timestamp_ms=timestamp_ms,
                )
            )
            idx += 3

        self.frame_count += 1
        return LidarFrame(
            speed_deg_per_s=float(speed_raw),
            start_angle_deg_cw=start_deg,
            end_angle_deg_cw=end_deg,
            timestamp_ms=timestamp_ms,
            points=points,
            crc_ok=crc_ok,
        )


class SerialLD20:
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 230400, timeout_s: float = 0.1, *, mount_yaw_offset_deg: float = 0.0, check_crc: bool = True) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.parser = LD20Parser(mount_yaw_offset_deg=mount_yaw_offset_deg, check_crc=check_crc)
        self.ser = serial.Serial(port, baud, timeout=timeout_s)

    def close(self) -> None:
        self.ser.close()

    def read_available_frames(self, max_bytes: int = 512) -> list[LidarFrame]:
        waiting = self.ser.in_waiting if hasattr(self.ser, "in_waiting") else 0
        n = max(1, min(max_bytes, waiting or max_bytes))
        return self.parser.feed(self.ser.read(n))

    def frames(self) -> Iterator[LidarFrame]:
        while True:
            for frame in self.read_available_frames():
                yield frame


def flatten_points(frames: Iterable[LidarFrame]) -> list[LidarPoint]:
    pts: list[LidarPoint] = []
    for frame in frames:
        pts.extend(frame.points)
    return pts
