from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class EncoderReader(Protocol):
    def read_left_ticks(self) -> int: ...
    def read_right_ticks(self) -> int: ...


@dataclass
class EncoderConfig:
    ticks_per_wheel_revolution: int
    left_channel_a: int | None = None
    left_channel_b: int | None = None
    right_channel_a: int | None = None
    right_channel_b: int | None = None


class DummyEncoderReader:
    """Placeholder until real encoders are installed."""

    def __init__(self) -> None:
        self.left_ticks = 0
        self.right_ticks = 0

    def read_left_ticks(self) -> int:
        return self.left_ticks

    def read_right_ticks(self) -> int:
        return self.right_ticks
