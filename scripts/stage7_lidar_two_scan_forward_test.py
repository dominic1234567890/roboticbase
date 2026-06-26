#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from tcr_minibot.odometry.differential_odometry import Pose2D
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, EdgeSegment, MappingScan, RoomMapper, XYPoint
from tcr_minibot.perception.lidar_filters import PointCloudFilterConfig, aggregate_one_scan
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.utils.config import load_config


def collect_filtered_scan(
    lidar: SerialLD20,
    *,
    args: argparse.Namespace,
    cfg: dict,
    filter_config: PointCloudFilterConfig,
    label: str,
) -> list[LidarPoint]:
    print(f"Collecting {label}...")
    points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
    return clean_lidar_points(points, cfg=cfg, filter_config=filter_config, label=label)


def write_point_cloud_csv(path: Path, scans: list[tuple[str, MappingScan]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_id", "x_m", "y_m"])
        for scan_id, scan in scans:
            for point in scan.points:
                writer.writerow([scan_id, f"{point.x_m:.4f}", f"{point.y_m:.4f}"])


def write_edges_csv(path: Path, scans: list[tuple[str, MappingScan]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_id", "start_x_m", "start_y_m", "end_x_m", "end_y_m", "length_m", "heading_deg"])
        for scan_id, scan in scans:
            for segment in scan.edge_segments:
                writer.writerow(
                    [
                        scan_id,
                        f"{segment.start_x_m:.4f}",
                        f"{segment.start_y_m:.4f}",
                        f"{segment.end_x_m:.4f}",
                        f"{segment.end_y_m:.4f}",
                        f"{segment.length_m:.4f}",
                        f"{segment.heading_deg:.2f}",
                    ]
                )


def plot_two_scan_map(
    mapper: RoomMapper,
    *,
    scan_a: MappingScan,
    scan_b: MappingScan,
    pose_b: Pose2D,
    save_path: Path,
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(
        mapper.grid.as_display_image(),
        extent=mapper.grid.image_extent_m(),
        origin="upper",
        cmap="gray",
        vmin=0,
        vmax=255,
    )

    _scatter_scan(ax, scan_a.points, color="tab:blue", label="scan 1")
    _scatter_scan(ax, scan_b.points, color="tab:orange", label="scan 2 shifted")
    _plot_edges(ax, scan_a.edge_segments, color="tab:blue")
    _plot_edges(ax, scan_b.edge_segments, color="tab:orange")

    ax.scatter([0.0], [0.0], marker="x", s=75, color="black", label="pose 1")
    ax.scatter([pose_b.x_m], [pose_b.y_m], marker="+", s=95, color="black", label="pose 2")
    ax.arrow(
        pose_b.x_m,
        pose_b.y_m,
        0.25 * math.cos(pose_b.heading_rad),
        0.25 * math.sin(pose_b.heading_rad),
        head_width=0.05,
        length_includes_head=True,
        color="black",
        alpha=0.8,
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward from first scan (m)")
    ax.set_ylabel("y left from first scan (m)")
    ax.set_title("Two-scan measured forward translation test")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=170)
    print(f"saved {save_path}")


def _scatter_scan(ax, points: list[XYPoint], *, color: str, label: str) -> None:
    if not points:
        return
    ax.scatter([p.x_m for p in points], [p.y_m for p in points], s=3, alpha=0.32, color=color, label=label)


def _plot_edges(ax, segments: list[EdgeSegment], *, color: str) -> None:
    for segment in segments:
        ax.plot(
            [segment.start_x_m, segment.end_x_m],
            [segment.start_y_m, segment.end_y_m],
            linewidth=1.6,
            alpha=0.8,
            color=color,
        )


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})

    ap = argparse.ArgumentParser(description="Stage 7: compose two LiDAR scans using a measured forward move")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--forward-m", type=float, default=mapping_cfg.get("two_scan_test_forward_m", 0.30))
    ap.add_argument("--heading-deg", type=float, default=0.0, help="Measured heading change before scan 2")
    ap.add_argument("--min-points", type=int, default=420, help="LiDAR points to collect per approximate scan")
    ap.add_argument("--save", default="data/captures/lidar_two_scan_forward_test.png")
    ap.add_argument("--point-cloud-csv", default=None)
    ap.add_argument("--edges-csv", default=None)
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--no-prompt", action="store_true", help="Do not wait for Enter between scans")
    ap.add_argument("--max-gap-m", type=float, default=mapping_cfg.get("edge_max_neighbor_gap_m", 0.18))
    ap.add_argument("--max-line-error-m", type=float, default=mapping_cfg.get("edge_max_line_error_m", 0.035))
    add_lidar_filter_args(ap, mapping_cfg)
    args = ap.parse_args()
    filter_config = filter_config_from_args(args)

    grid = OccupancyGrid(
        size_m=float(mapping_cfg.get("size_m", 6.0)),
        cell_size_m=float(mapping_cfg.get("cell_size_m", 0.05)),
    )
    mapper = RoomMapper(
        grid=grid,
        edge_config=EdgeMappingConfig(
            max_neighbor_gap_m=args.max_gap_m,
            max_line_error_m=args.max_line_error_m,
            min_points_per_segment=mapping_cfg.get("edge_min_segment_points", 8),
            min_segment_length_m=mapping_cfg.get("edge_min_segment_length_m", 0.25),
        ),
    )

    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )

    print("This script does not drive motors.")
    print("Measure the requested forward move from the LiDAR center, not the front caster.")
    try:
        first_points = collect_filtered_scan(lidar, args=args, cfg=cfg, filter_config=filter_config, label="scan 1")
        if not args.no_prompt:
            input(f"Move the robot forward {args.forward_m:.3f} m, keep the heading straight, then press Enter...")
        second_points = collect_filtered_scan(lidar, args=args, cfg=cfg, filter_config=filter_config, label="scan 2")
    finally:
        lidar.close()

    pose_b = Pose2D(x_m=args.forward_m, y_m=0.0, heading_rad=math.radians(args.heading_deg))
    scan_a = mapper.add_scan(first_points, pose=Pose2D())
    scan_b = mapper.add_scan(second_points, pose=pose_b)

    print(
        f"composed scans: scan1_points={len(scan_a.points)} scan2_points={len(scan_b.points)} "
        f"scan1_edges={len(scan_a.edge_segments)} scan2_edges={len(scan_b.edge_segments)}"
    )

    scans = [("scan_1", scan_a), ("scan_2", scan_b)]
    if args.point_cloud_csv:
        write_point_cloud_csv(Path(args.point_cloud_csv), scans)
        print(f"saved {args.point_cloud_csv}")
    if args.edges_csv:
        write_edges_csv(Path(args.edges_csv), scans)
        print(f"saved {args.edges_csv}")

    plot_two_scan_map(mapper, scan_a=scan_a, scan_b=scan_b, pose_b=pose_b, save_path=Path(args.save))


if __name__ == "__main__":
    main()
