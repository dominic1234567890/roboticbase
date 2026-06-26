from __future__ import annotations

import math


def wrap_deg(angle_deg: float) -> float:
    """Wrap angle to [0, 360)."""
    return angle_deg % 360.0


def wrap_signed_deg(angle_deg: float) -> float:
    """Wrap angle to [-180, 180)."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def deg_to_rad(angle_deg: float) -> float:
    return angle_deg * math.pi / 180.0


def polar_to_xy(distance_m: float, bearing_deg_ccw: float) -> tuple[float, float]:
    """Robot convention: +x is forward, +y is left, angle positive CCW."""
    theta = deg_to_rad(bearing_deg_ccw)
    return distance_m * math.cos(theta), distance_m * math.sin(theta)


def ld20_clockwise_to_robot_ccw(raw_angle_deg_cw: float, mount_yaw_offset_deg: float = 0.0) -> float:
    """
    LD20 docs define angle increasing clockwise. For robot math we usually want CCW.

    mount_yaw_offset_deg is the correction that makes 0 degrees point along robot +x.
    Example: if the LiDAR's 0-degree mark points 90 degrees right of robot-front,
    set this after a physical calibration test.
    """
    return wrap_signed_deg(-(raw_angle_deg_cw + mount_yaw_offset_deg))
