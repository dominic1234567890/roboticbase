from dataclasses import dataclass

from tcr_minibot.perception.lidar_filters import (
    PointCloudFilterConfig,
    apply_point_cloud_filters,
    passthrough_filter,
    radius_outlier_removal,
    statistical_outlier_removal,
    voxel_grid_downsample,
)


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float
    bearing_deg: float | None = None


def test_passthrough_filter_limits_axes_and_wraparound_bearing():
    points = [
        Point(1.0, 0.0, 350.0),
        Point(1.0, 0.1, 10.0),
        Point(1.0, 0.2, 180.0),
        Point(-1.0, 0.0, 0.0),
    ]

    filtered = passthrough_filter(
        points,
        min_x_m=0.0,
        min_bearing_deg=340.0,
        max_bearing_deg=20.0,
    )

    assert filtered == points[:2]


def test_voxel_grid_downsample_keeps_one_representative_per_cell():
    points = [
        Point(0.01, 0.01),
        Point(0.03, 0.02),
        Point(0.21, 0.01),
    ]

    filtered = voxel_grid_downsample(points, leaf_size_m=0.1)

    assert len(filtered) == 2
    assert filtered[-1] == points[-1]


def test_radius_outlier_removal_drops_isolated_points():
    points = [
        Point(0.0, 0.0),
        Point(0.03, 0.0),
        Point(-0.03, 0.0),
        Point(1.0, 1.0),
    ]

    filtered = radius_outlier_removal(points, radius_m=0.05, min_neighbors=1)

    assert filtered == points[:3]


def test_statistical_outlier_removal_drops_far_point():
    points = [
        Point(0.0, 0.0),
        Point(0.05, 0.0),
        Point(0.10, 0.0),
        Point(0.15, 0.0),
        Point(2.0, 2.0),
    ]

    filtered = statistical_outlier_removal(points, mean_k=2, std_ratio=1.0)

    assert points[-1] not in filtered
    assert len(filtered) == 4


def test_apply_point_cloud_filters_combines_enabled_filters():
    points = [
        Point(0.0, 0.0),
        Point(0.01, 0.01),
        Point(0.20, 0.0),
        Point(-0.20, 0.0),
    ]
    config = PointCloudFilterConfig(
        passthrough_min_x_m=0.0,
        voxel_leaf_size_m=0.1,
    )

    filtered = apply_point_cloud_filters(points, config)

    assert len(filtered) == 2
    assert all(point.x_m >= 0.0 for point in filtered)
