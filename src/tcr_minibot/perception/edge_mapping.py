from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable

import numpy as np

from tcr_minibot.odometry.differential_odometry import Pose2D
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import LidarPoint
from tcr_minibot.utils.geometry import wrap_signed_deg


@dataclass(frozen=True)
class XYPoint:
    x_m: float
    y_m: float
    bearing_deg: float | None = None


@dataclass(frozen=True)
class EdgeSegment:
    start_x_m: float
    start_y_m: float
    end_x_m: float
    end_y_m: float
    point_count: int
    rms_error_m: float

    @property
    def length_m(self) -> float:
        return math.hypot(self.end_x_m - self.start_x_m, self.end_y_m - self.start_y_m)

    @property
    def heading_deg(self) -> float:
        return wrap_signed_deg(math.degrees(math.atan2(self.end_y_m - self.start_y_m, self.end_x_m - self.start_x_m)))

    @property
    def midpoint_m(self) -> tuple[float, float]:
        return ((self.start_x_m + self.end_x_m) / 2.0, (self.start_y_m + self.end_y_m) / 2.0)


@dataclass(frozen=True)
class EdgeMappingConfig:
    max_neighbor_gap_m: float = 0.18
    max_line_error_m: float = 0.035
    min_points_per_segment: int = 8
    min_segment_length_m: float = 0.25
    max_split_depth: int = 8


@dataclass(frozen=True)
class MappingScan:
    points: list[XYPoint]
    edge_segments: list[EdgeSegment]
    pose: Pose2D = field(default_factory=Pose2D)


@dataclass(frozen=True)
class _LineFit:
    segment: EdgeSegment
    max_error_m: float
    max_error_index: int


@dataclass
class RoomMapper:
    grid: OccupancyGrid = field(default_factory=OccupancyGrid)
    edge_config: EdgeMappingConfig = field(default_factory=EdgeMappingConfig)
    point_cloud: list[XYPoint] = field(default_factory=list)
    edge_segments: list[EdgeSegment] = field(default_factory=list)
    scan_count: int = 0

    def add_scan(self, points: Iterable[LidarPoint | XYPoint], pose: Pose2D | None = None) -> MappingScan:
        pose = pose or Pose2D()
        local_points = coerce_xy_points(points)
        world_points = transform_points(local_points, pose)
        self.grid.add_xy_points(((p.x_m, p.y_m) for p in world_points), origin_m=(pose.x_m, pose.y_m))

        local_segments = extract_edge_segments(local_points, self.edge_config)
        world_segments = [transform_segment(segment, pose) for segment in local_segments]

        self.point_cloud.extend(world_points)
        self.edge_segments.extend(world_segments)
        self.scan_count += 1
        return MappingScan(points=world_points, edge_segments=world_segments, pose=copy_pose(pose))


def coerce_xy_points(points: Iterable[LidarPoint | XYPoint]) -> list[XYPoint]:
    out: list[XYPoint] = []
    for point in points:
        x_m = float(point.x_m)
        y_m = float(point.y_m)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            continue
        bearing = point.bearing_deg
        out.append(XYPoint(x_m=x_m, y_m=y_m, bearing_deg=bearing))
    return out


def transform_points(points: Iterable[XYPoint], pose: Pose2D) -> list[XYPoint]:
    cos_h = math.cos(pose.heading_rad)
    sin_h = math.sin(pose.heading_rad)
    transformed: list[XYPoint] = []
    for point in points:
        x_w = pose.x_m + point.x_m * cos_h - point.y_m * sin_h
        y_w = pose.y_m + point.x_m * sin_h + point.y_m * cos_h
        transformed.append(XYPoint(x_m=x_w, y_m=y_w, bearing_deg=None))
    return transformed


def transform_segment(segment: EdgeSegment, pose: Pose2D) -> EdgeSegment:
    start = transform_points([XYPoint(segment.start_x_m, segment.start_y_m)], pose)[0]
    end = transform_points([XYPoint(segment.end_x_m, segment.end_y_m)], pose)[0]
    return EdgeSegment(
        start_x_m=start.x_m,
        start_y_m=start.y_m,
        end_x_m=end.x_m,
        end_y_m=end.y_m,
        point_count=segment.point_count,
        rms_error_m=segment.rms_error_m,
    )


def split_scan_clusters(points: Iterable[LidarPoint | XYPoint], config: EdgeMappingConfig = EdgeMappingConfig()) -> list[list[XYPoint]]:
    sorted_points = sorted(coerce_xy_points(points), key=_bearing_sort_key)
    if not sorted_points:
        return []

    clusters: list[list[XYPoint]] = [[sorted_points[0]]]
    for previous, current in zip(sorted_points, sorted_points[1:]):
        if _distance(previous, current) > config.max_neighbor_gap_m:
            clusters.append([current])
        else:
            clusters[-1].append(current)

    if len(clusters) > 1 and _distance(clusters[0][0], clusters[-1][-1]) <= config.max_neighbor_gap_m:
        clusters[0] = clusters[-1] + clusters[0]
        clusters.pop()

    return [cluster for cluster in clusters if len(cluster) >= config.min_points_per_segment]


def extract_edge_segments(
    points: Iterable[LidarPoint | XYPoint],
    config: EdgeMappingConfig = EdgeMappingConfig(),
) -> list[EdgeSegment]:
    segments: list[EdgeSegment] = []
    for cluster in split_scan_clusters(points, config):
        segments.extend(_split_and_fit(cluster, config=config, depth=0))
    return sorted(segments, key=lambda segment: segment.length_m, reverse=True)


def _bearing_sort_key(point: XYPoint) -> float:
    if point.bearing_deg is not None:
        return point.bearing_deg % 360.0
    return math.degrees(math.atan2(point.y_m, point.x_m)) % 360.0


def _distance(a: XYPoint, b: XYPoint) -> float:
    return math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)


def _split_and_fit(points: list[XYPoint], *, config: EdgeMappingConfig, depth: int) -> list[EdgeSegment]:
    if len(points) < config.min_points_per_segment:
        return []

    fit = _fit_line(points)
    if fit.segment.length_m < config.min_segment_length_m:
        return []

    if fit.max_error_m <= config.max_line_error_m or depth >= config.max_split_depth:
        return [fit.segment]

    split_idx = fit.max_error_index
    left_count = split_idx + 1
    right_count = len(points) - split_idx
    if left_count < config.min_points_per_segment or right_count < config.min_points_per_segment:
        if fit.segment.rms_error_m <= config.max_line_error_m:
            return [fit.segment]
        return []

    left = _split_and_fit(points[: split_idx + 1], config=config, depth=depth + 1)
    right = _split_and_fit(points[split_idx:], config=config, depth=depth + 1)
    return left + right


def _fit_line(points: list[XYPoint]) -> _LineFit:
    xy = np.array([(point.x_m, point.y_m) for point in points], dtype=float)
    centroid = xy.mean(axis=0)
    centered = xy - centroid

    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    direction = vh[0]
    normal = np.array([-direction[1], direction[0]])

    projections = centered @ direction
    distances = np.abs(centered @ normal)
    start = centroid + direction * projections.min()
    end = centroid + direction * projections.max()

    max_error_index = int(np.argmax(distances))
    rms_error = float(math.sqrt(np.mean(distances**2)))
    return _LineFit(
        segment=EdgeSegment(
            start_x_m=float(start[0]),
            start_y_m=float(start[1]),
            end_x_m=float(end[0]),
            end_y_m=float(end[1]),
            point_count=len(points),
            rms_error_m=rms_error,
        ),
        max_error_m=float(distances[max_error_index]),
        max_error_index=max_error_index,
    )


def copy_pose(pose: Pose2D) -> Pose2D:
    return Pose2D(x_m=pose.x_m, y_m=pose.y_m, heading_rad=pose.heading_rad)
