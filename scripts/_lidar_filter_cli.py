from __future__ import annotations

import argparse
from typing import Any

from tcr_minibot.perception.lidar_filters import PointCloudFilterConfig, apply_point_cloud_filters, valid_points
from tcr_minibot.sensors.lidar_ld20 import LidarPoint


def add_lidar_filter_args(ap: argparse.ArgumentParser, mapping_cfg: dict[str, Any]) -> None:
    filters = ap.add_argument_group("point cloud filters")
    filters.add_argument("--voxel-size-m", type=float, default=mapping_cfg.get("filter_voxel_leaf_size_m"))
    filters.add_argument("--sor-mean-k", type=int, default=mapping_cfg.get("filter_sor_mean_k", 0))
    filters.add_argument("--sor-std-ratio", type=float, default=mapping_cfg.get("filter_sor_std_ratio", 1.0))
    filters.add_argument("--radius-outlier-radius-m", type=float, default=mapping_cfg.get("filter_radius_outlier_radius_m"))
    filters.add_argument(
        "--radius-outlier-min-neighbors",
        type=int,
        default=mapping_cfg.get("filter_radius_outlier_min_neighbors", 0),
    )
    filters.add_argument("--pass-x-min-m", type=float, default=mapping_cfg.get("filter_passthrough_min_x_m"))
    filters.add_argument("--pass-x-max-m", type=float, default=mapping_cfg.get("filter_passthrough_max_x_m"))
    filters.add_argument("--pass-y-min-m", type=float, default=mapping_cfg.get("filter_passthrough_min_y_m"))
    filters.add_argument("--pass-y-max-m", type=float, default=mapping_cfg.get("filter_passthrough_max_y_m"))
    filters.add_argument("--pass-range-min-m", type=float, default=mapping_cfg.get("filter_passthrough_min_range_m"))
    filters.add_argument("--pass-range-max-m", type=float, default=mapping_cfg.get("filter_passthrough_max_range_m"))
    filters.add_argument("--pass-bearing-min-deg", type=float, default=mapping_cfg.get("filter_passthrough_min_bearing_deg"))
    filters.add_argument("--pass-bearing-max-deg", type=float, default=mapping_cfg.get("filter_passthrough_max_bearing_deg"))


def filter_config_from_args(args: argparse.Namespace) -> PointCloudFilterConfig:
    return PointCloudFilterConfig(
        passthrough_min_x_m=args.pass_x_min_m,
        passthrough_max_x_m=args.pass_x_max_m,
        passthrough_min_y_m=args.pass_y_min_m,
        passthrough_max_y_m=args.pass_y_max_m,
        passthrough_min_range_m=args.pass_range_min_m,
        passthrough_max_range_m=args.pass_range_max_m,
        passthrough_min_bearing_deg=args.pass_bearing_min_deg,
        passthrough_max_bearing_deg=args.pass_bearing_max_deg,
        radius_outlier_radius_m=args.radius_outlier_radius_m,
        radius_outlier_min_neighbors=args.radius_outlier_min_neighbors,
        sor_mean_k=args.sor_mean_k,
        sor_std_ratio=args.sor_std_ratio,
        voxel_leaf_size_m=args.voxel_size_m,
    )


def clean_lidar_points(
    points: list[LidarPoint],
    *,
    cfg: dict[str, Any],
    filter_config: PointCloudFilterConfig,
    label: str,
) -> list[LidarPoint]:
    raw_count = len(points)
    ranged = valid_points(
        points,
        min_m=cfg["lidar"].get("min_distance_m", 0.08),
        max_m=cfg["lidar"].get("max_distance_m", 8.0),
        min_confidence=cfg["lidar"].get("min_confidence", 0),
    )
    filtered = apply_point_cloud_filters(ranged, filter_config)
    print(f"{label}: raw={raw_count} valid={len(ranged)} filtered={len(filtered)}")
    return filtered
