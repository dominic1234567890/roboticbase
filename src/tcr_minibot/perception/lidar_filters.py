from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Protocol, TypeVar

from tcr_minibot.sensors.lidar_ld20 import LidarPoint
from tcr_minibot.utils.geometry import wrap_signed_deg


class XYPointLike(Protocol):
    x_m: float
    y_m: float


TPoint = TypeVar("TPoint", bound=XYPointLike)


@dataclass(frozen=True)
class ZoneDistances:
    front_min_m: float | None
    left_min_m: float | None
    right_min_m: float | None
    rear_min_m: float | None

    @property
    def front_blocked(self) -> bool:
        return self.front_min_m is not None and self.front_min_m < 0.35


@dataclass(frozen=True)
class PointCloudFilterConfig:
    passthrough_min_x_m: float | None = None
    passthrough_max_x_m: float | None = None
    passthrough_min_y_m: float | None = None
    passthrough_max_y_m: float | None = None
    passthrough_min_range_m: float | None = None
    passthrough_max_range_m: float | None = None
    passthrough_min_bearing_deg: float | None = None
    passthrough_max_bearing_deg: float | None = None
    radius_outlier_radius_m: float | None = None
    radius_outlier_min_neighbors: int = 0
    sor_mean_k: int = 0
    sor_std_ratio: float = 1.0
    voxel_leaf_size_m: float | None = None


def valid_points(points: Iterable[LidarPoint], *, min_m: float = 0.08, max_m: float = 8.0, min_confidence: int = 0) -> list[LidarPoint]:
    out: list[LidarPoint] = []
    for p in points:
        if not math.isfinite(p.distance_m):
            continue
        if p.distance_m < min_m or p.distance_m > max_m:
            continue
        if p.confidence < min_confidence:
            continue
        out.append(p)
    return out


def points_in_angle_window(points: Iterable[LidarPoint], center_deg: float, half_width_deg: float) -> list[LidarPoint]:
    return [p for p in points if abs(wrap_signed_deg(p.bearing_deg - center_deg)) <= half_width_deg]


def min_distance(points: Iterable[LidarPoint]) -> float | None:
    distances = [p.distance_m for p in points]
    return min(distances) if distances else None


def compute_zone_distances(points: Iterable[LidarPoint], *, front_half_angle_deg: float = 25.0, side_half_angle_deg: float = 20.0) -> ZoneDistances:
    pts = list(points)
    return ZoneDistances(
        front_min_m=min_distance(points_in_angle_window(pts, 0.0, front_half_angle_deg)),
        left_min_m=min_distance(points_in_angle_window(pts, 90.0, side_half_angle_deg)),
        right_min_m=min_distance(points_in_angle_window(pts, -90.0, side_half_angle_deg)),
        rear_min_m=min_distance(points_in_angle_window(pts, 180.0, side_half_angle_deg)),
    )


def aggregate_one_scan(frame_iter, min_points: int = 360, max_frames: int = 120) -> list[LidarPoint]:
    """Collect enough packets to approximate one 360-degree scan."""
    points: list[LidarPoint] = []
    for i, frame in enumerate(frame_iter):
        points.extend(frame.points)
        if len(points) >= min_points or i + 1 >= max_frames:
            return points
    return points


def apply_point_cloud_filters(points: Iterable[TPoint], config: PointCloudFilterConfig) -> list[TPoint]:
    filtered = passthrough_filter(
        points,
        min_x_m=config.passthrough_min_x_m,
        max_x_m=config.passthrough_max_x_m,
        min_y_m=config.passthrough_min_y_m,
        max_y_m=config.passthrough_max_y_m,
        min_range_m=config.passthrough_min_range_m,
        max_range_m=config.passthrough_max_range_m,
        min_bearing_deg=config.passthrough_min_bearing_deg,
        max_bearing_deg=config.passthrough_max_bearing_deg,
    )
    filtered = radius_outlier_removal(
        filtered,
        radius_m=config.radius_outlier_radius_m,
        min_neighbors=config.radius_outlier_min_neighbors,
    )
    filtered = statistical_outlier_removal(
        filtered,
        mean_k=config.sor_mean_k,
        std_ratio=config.sor_std_ratio,
    )
    return voxel_grid_downsample(filtered, leaf_size_m=config.voxel_leaf_size_m)


def passthrough_filter(
    points: Iterable[TPoint],
    *,
    min_x_m: float | None = None,
    max_x_m: float | None = None,
    min_y_m: float | None = None,
    max_y_m: float | None = None,
    min_range_m: float | None = None,
    max_range_m: float | None = None,
    min_bearing_deg: float | None = None,
    max_bearing_deg: float | None = None,
) -> list[TPoint]:
    out: list[TPoint] = []
    for point in points:
        x_m = float(point.x_m)
        y_m = float(point.y_m)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            continue
        if min_x_m is not None and x_m < min_x_m:
            continue
        if max_x_m is not None and x_m > max_x_m:
            continue
        if min_y_m is not None and y_m < min_y_m:
            continue
        if max_y_m is not None and y_m > max_y_m:
            continue

        range_m = _point_range_m(point)
        if min_range_m is not None and range_m < min_range_m:
            continue
        if max_range_m is not None and range_m > max_range_m:
            continue

        if min_bearing_deg is not None or max_bearing_deg is not None:
            bearing_deg = _point_bearing_deg(point)
            if min_bearing_deg is not None and max_bearing_deg is not None:
                if not _bearing_in_window(bearing_deg, min_bearing_deg, max_bearing_deg):
                    continue
            elif min_bearing_deg is not None and bearing_deg < min_bearing_deg:
                continue
            elif max_bearing_deg is not None and bearing_deg > max_bearing_deg:
                continue

        out.append(point)
    return out


def voxel_grid_downsample(points: Iterable[TPoint], leaf_size_m: float | None) -> list[TPoint]:
    pts = list(points)
    if leaf_size_m is None or leaf_size_m <= 0.0 or len(pts) < 2:
        return pts

    voxels: dict[tuple[int, int], list[tuple[int, TPoint]]] = {}
    for idx, point in enumerate(pts):
        key = (math.floor(point.x_m / leaf_size_m), math.floor(point.y_m / leaf_size_m))
        voxels.setdefault(key, []).append((idx, point))

    representatives: list[tuple[int, TPoint]] = []
    for members in voxels.values():
        centroid_x = sum(point.x_m for _, point in members) / len(members)
        centroid_y = sum(point.y_m for _, point in members) / len(members)
        representatives.append(
            min(
                members,
                key=lambda item: (item[1].x_m - centroid_x) ** 2 + (item[1].y_m - centroid_y) ** 2,
            )
        )

    representatives.sort(key=lambda item: item[0])
    return [point for _, point in representatives]


def radius_outlier_removal(points: Iterable[TPoint], *, radius_m: float | None, min_neighbors: int) -> list[TPoint]:
    pts = list(points)
    if radius_m is None or radius_m <= 0.0 or min_neighbors <= 0 or len(pts) < 2:
        return pts

    radius_sq = radius_m * radius_m
    out: list[TPoint] = []
    for idx, point in enumerate(pts):
        neighbors = 0
        for other_idx, other in enumerate(pts):
            if other_idx == idx:
                continue
            if _distance_sq(point, other) <= radius_sq:
                neighbors += 1
                if neighbors >= min_neighbors:
                    out.append(point)
                    break
    return out


def statistical_outlier_removal(points: Iterable[TPoint], *, mean_k: int, std_ratio: float = 1.0) -> list[TPoint]:
    pts = list(points)
    if mean_k <= 0 or len(pts) <= mean_k:
        return pts

    k = min(mean_k, len(pts) - 1)
    mean_distances: list[float] = []
    for idx, point in enumerate(pts):
        distances = [
            math.sqrt(_distance_sq(point, other))
            for other_idx, other in enumerate(pts)
            if other_idx != idx
        ]
        distances.sort()
        mean_distances.append(sum(distances[:k]) / k)

    global_mean = sum(mean_distances) / len(mean_distances)
    variance = sum((value - global_mean) ** 2 for value in mean_distances) / len(mean_distances)
    threshold = global_mean + max(0.0, std_ratio) * math.sqrt(variance)
    return [point for point, mean_distance in zip(pts, mean_distances) if mean_distance <= threshold]


def _point_range_m(point: XYPointLike) -> float:
    distance_m = getattr(point, "distance_m", None)
    if isinstance(distance_m, (int, float)) and math.isfinite(distance_m):
        return float(distance_m)
    return math.hypot(point.x_m, point.y_m)


def _point_bearing_deg(point: XYPointLike) -> float:
    bearing_deg = getattr(point, "bearing_deg", None)
    if isinstance(bearing_deg, (int, float)) and math.isfinite(bearing_deg):
        return float(bearing_deg)
    return wrap_signed_deg(math.degrees(math.atan2(point.y_m, point.x_m)))


def _bearing_in_window(bearing_deg: float, min_bearing_deg: float, max_bearing_deg: float) -> bool:
    bearing = bearing_deg % 360.0
    low = min_bearing_deg % 360.0
    high = max_bearing_deg % 360.0
    if low <= high:
        return low <= bearing <= high
    return bearing >= low or bearing <= high


def _distance_sq(a: XYPointLike, b: XYPointLike) -> float:
    return (a.x_m - b.x_m) ** 2 + (a.y_m - b.y_m) ** 2
