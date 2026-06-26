from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


@dataclass
class DifferentialOdometry:
    wheel_radius_m: float
    wheel_track_m: float
    ticks_per_rev: int
    pose: Pose2D = field(default_factory=Pose2D)
    last_left_ticks: int | None = None
    last_right_ticks: int | None = None

    def ticks_to_distance(self, ticks: int) -> float:
        wheel_circumference = 2.0 * math.pi * self.wheel_radius_m
        return ticks / self.ticks_per_rev * wheel_circumference

    def update_from_ticks(self, left_ticks: int, right_ticks: int) -> Pose2D:
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            return self.pose

        d_left = self.ticks_to_distance(left_ticks - self.last_left_ticks)
        d_right = self.ticks_to_distance(right_ticks - self.last_right_ticks)
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        d_center = (d_left + d_right) / 2.0
        d_heading = (d_right - d_left) / self.wheel_track_m

        mid_heading = self.pose.heading_rad + d_heading / 2.0
        self.pose.x_m += d_center * math.cos(mid_heading)
        self.pose.y_m += d_center * math.sin(mid_heading)
        self.pose.heading_rad = (self.pose.heading_rad + d_heading + math.pi) % (2.0 * math.pi) - math.pi
        return self.pose
