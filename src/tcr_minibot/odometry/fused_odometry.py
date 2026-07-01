from __future__ import annotations

from dataclasses import dataclass, field
import math
from time import monotonic
from typing import Protocol

from tcr_minibot.odometry.differential_odometry import Pose2D


class HeadingRateSource(Protocol):
    """Future gyro/IMU hook: return z-axis yaw rate in radians/second."""

    def read_yaw_rate_radps(self) -> float: ...


class AbsoluteHeadingSource(Protocol):
    """Future IMU hook: return robot heading in radians, if you add such a sensor."""

    def read_heading_rad(self) -> float: ...


@dataclass(frozen=True)
class WheelOdometryConfig:
    wheel_radius_m: float = 0.0325
    wheel_track_m: float = 0.1143
    ticks_per_wheel_rev: int = 40

    def ticks_to_distance_m(self, ticks: int) -> float:
        if self.ticks_per_wheel_rev <= 0:
            raise ValueError("ticks_per_wheel_rev must be greater than zero")
        return (ticks / self.ticks_per_wheel_rev) * (2.0 * math.pi * self.wheel_radius_m)


@dataclass(frozen=True)
class FusionConfig:
    """
    Heading fusion knobs.

    gyro_delta_weight=0.0 means wheel odometry controls heading for now.
    Later, with a gyro z-rate source, try 0.2 to 0.7 after calibration.
    absolute_heading_weight is for a future absolute-heading IMU/magnetometer style source.
    """

    gyro_delta_weight: float = 0.0
    absolute_heading_weight: float = 0.0


@dataclass(frozen=True)
class OdometryUpdate:
    pose: Pose2D
    left_delta_ticks: int
    right_delta_ticks: int
    left_distance_m: float
    right_distance_m: float
    center_distance_m: float
    wheel_heading_delta_rad: float
    used_heading_delta_rad: float
    dt_s: float | None
    heading_source: str


@dataclass
class WheelGyroOdometry:
    """
    Differential-drive odometry with a clean future hook for gyro/IMU fusion.

    Today you can run it with only wheel encoders. When a gyro is added, pass a
    yaw rate in rad/s into update(..., gyro_yaw_rate_radps=...). If you later add
    an IMU with absolute heading, pass absolute_heading_rad instead.
    """

    wheel_config: WheelOdometryConfig
    fusion_config: FusionConfig = field(default_factory=FusionConfig)
    pose: Pose2D = field(default_factory=Pose2D)
    last_left_ticks: int | None = None
    last_right_ticks: int | None = None
    last_time_s: float | None = None

    def reset(
        self,
        *,
        pose: Pose2D | None = None,
        left_ticks: int | None = None,
        right_ticks: int | None = None,
        now_s: float | None = None,
    ) -> None:
        self.pose = copy_pose(pose or Pose2D())
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time_s = monotonic() if now_s is None else now_s

    def update(
        self,
        left_ticks: int,
        right_ticks: int,
        *,
        now_s: float | None = None,
        gyro_yaw_rate_radps: float | None = None,
        absolute_heading_rad: float | None = None,
    ) -> OdometryUpdate:
        now = monotonic() if now_s is None else now_s
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time_s = now
            return OdometryUpdate(
                pose=copy_pose(self.pose),
                left_delta_ticks=0,
                right_delta_ticks=0,
                left_distance_m=0.0,
                right_distance_m=0.0,
                center_distance_m=0.0,
                wheel_heading_delta_rad=0.0,
                used_heading_delta_rad=0.0,
                dt_s=None,
                heading_source="init",
            )

        dt_s = None if self.last_time_s is None else max(0.0, now - self.last_time_s)
        left_delta_ticks = left_ticks - self.last_left_ticks
        right_delta_ticks = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time_s = now

        left_distance_m = self.wheel_config.ticks_to_distance_m(left_delta_ticks)
        right_distance_m = self.wheel_config.ticks_to_distance_m(right_delta_ticks)
        center_distance_m = (left_distance_m + right_distance_m) / 2.0
        wheel_heading_delta_rad = (right_distance_m - left_distance_m) / self.wheel_config.wheel_track_m

        used_heading_delta_rad = wheel_heading_delta_rad
        heading_source = "wheels"

        if gyro_yaw_rate_radps is not None and dt_s is not None and dt_s > 0.0:
            weight = clamp01(self.fusion_config.gyro_delta_weight)
            gyro_delta = gyro_yaw_rate_radps * dt_s
            used_heading_delta_rad = (1.0 - weight) * wheel_heading_delta_rad + weight * gyro_delta
            heading_source = f"wheels+gyro_rate:{weight:.2f}"

        new_heading = wrap_pi(self.pose.heading_rad + used_heading_delta_rad)
        if absolute_heading_rad is not None:
            weight = clamp01(self.fusion_config.absolute_heading_weight)
            new_heading = blend_angles_rad(new_heading, absolute_heading_rad, weight)
            used_heading_delta_rad = wrap_pi(new_heading - self.pose.heading_rad)
            heading_source = f"wheels+absolute_heading:{weight:.2f}"

        mid_heading = self.pose.heading_rad + used_heading_delta_rad / 2.0
        self.pose.x_m += center_distance_m * math.cos(mid_heading)
        self.pose.y_m += center_distance_m * math.sin(mid_heading)
        self.pose.heading_rad = wrap_pi(new_heading)

        return OdometryUpdate(
            pose=copy_pose(self.pose),
            left_delta_ticks=left_delta_ticks,
            right_delta_ticks=right_delta_ticks,
            left_distance_m=left_distance_m,
            right_distance_m=right_distance_m,
            center_distance_m=center_distance_m,
            wheel_heading_delta_rad=wheel_heading_delta_rad,
            used_heading_delta_rad=used_heading_delta_rad,
            dt_s=dt_s,
            heading_source=heading_source,
        )


def copy_pose(pose: Pose2D) -> Pose2D:
    return Pose2D(x_m=pose.x_m, y_m=pose.y_m, heading_rad=pose.heading_rad)


def wrap_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def blend_angles_rad(a_rad: float, b_rad: float, weight_b: float) -> float:
    """Return angle a blended toward angle b by weight_b, respecting wraparound."""

    w = clamp01(weight_b)
    return wrap_pi(a_rad + wrap_pi(b_rad - a_rad) * w)
