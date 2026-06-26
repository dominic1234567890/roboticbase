from __future__ import annotations

from dataclasses import replace
from typing import Iterable

from tcr_minibot.perception.vision_simple import Detection2D
from tcr_minibot.sensors.lidar_ld20 import LidarPoint
from tcr_minibot.utils.geometry import wrap_signed_deg


def pixel_x_to_bearing_deg(x_px: float, image_width_px: int, horizontal_fov_deg: float, camera_yaw_offset_deg: float = 0.0) -> float:
    """
    Convert image x-center to approximate bearing.

    Robot convention:
    - 0 deg is straight forward.
    - positive deg is left.
    Image convention:
    - x=0 is left side of image.
    """
    normalized = (x_px - image_width_px / 2) / (image_width_px / 2)
    # Image left should be positive robot bearing; image right negative.
    return wrap_signed_deg(-normalized * (horizontal_fov_deg / 2) + camera_yaw_offset_deg)


def nearest_lidar_range_for_bearing(points: Iterable[LidarPoint], bearing_deg: float, tolerance_deg: float = 3.0) -> float | None:
    candidates: list[tuple[float, float]] = []
    for p in points:
        err = abs(wrap_signed_deg(p.bearing_deg - bearing_deg))
        if err <= tolerance_deg:
            candidates.append((err, p.distance_m))
    if not candidates:
        return None
    # Use the closest range among nearby bearings. Useful for obstacle avoidance.
    return min(distance for _, distance in candidates)


def attach_lidar_ranges_to_detections(
    detections: list[Detection2D],
    lidar_points: list[LidarPoint],
    *,
    image_width_px: int,
    horizontal_fov_deg: float = 70.0,
    camera_yaw_offset_deg: float = 0.0,
    lidar_tolerance_deg: float = 4.0,
) -> list[Detection2D]:
    fused: list[Detection2D] = []
    for d in detections:
        bearing = pixel_x_to_bearing_deg(d.cx, image_width_px, horizontal_fov_deg, camera_yaw_offset_deg)
        range_m = nearest_lidar_range_for_bearing(lidar_points, bearing, lidar_tolerance_deg)
        fused.append(replace(d, bearing_deg=bearing, range_m=range_m))
    return fused
