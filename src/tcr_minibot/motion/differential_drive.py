from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WheelCommands:
    left: float
    right: float


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def arcade_to_wheel_power(forward: float, turn: float, max_abs: float = 100.0) -> WheelCommands:
    """
    Convert simple arcade drive commands to left/right motor power.

    Inputs can be any scale, but output is normalized to +/- max_abs.
    """
    left = forward - turn
    right = forward + turn
    biggest = max(abs(left), abs(right), max_abs)
    if biggest > max_abs:
        left *= max_abs / biggest
        right *= max_abs / biggest
    return WheelCommands(clamp(left, -max_abs, max_abs), clamp(right, -max_abs, max_abs))


def twist_to_wheel_speeds(linear_mps: float, angular_radps: float, wheel_track_m: float) -> tuple[float, float]:
    """Return left/right ground speeds in m/s for a differential drive."""
    half_track = wheel_track_m / 2.0
    left_mps = linear_mps - angular_radps * half_track
    right_mps = linear_mps + angular_radps * half_track
    return left_mps, right_mps
