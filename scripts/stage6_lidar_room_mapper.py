#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, EdgeSegment, RoomMapper, XYPoint
from tcr_minibot.perception.lidar_filters import aggregate_one_scan
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.utils.config import load_config


def write_point_cloud_csv(path: Path, points: list[XYPoint]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x_m", "y_m"])
        for point in points:
            writer.writerow([f"{point.x_m:.4f}", f"{point.y_m:.4f}"])


def write_edges_csv(path: Path, segments: list[EdgeSegment]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["start_x_m", "start_y_m", "end_x_m", "end_y_m", "length_m", "heading_deg"])
        for segment in segments:
            writer.writerow(
                [
                    f"{segment.start_x_m:.4f}",
                    f"{segment.start_y_m:.4f}",
                    f"{segment.end_x_m:.4f}",
                    f"{segment.end_y_m:.4f}",
                    f"{segment.length_m:.4f}",
                    f"{segment.heading_deg:.2f}",
                ]
            )


def plot_room_map(mapper: RoomMapper, save_path: Path) -> None:
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

    if mapper.point_cloud:
        ax.scatter(
            [p.x_m for p in mapper.point_cloud],
            [p.y_m for p in mapper.point_cloud],
            s=1,
            alpha=0.22,
            color="tab:blue",
            label="point cloud",
        )

    for idx, segment in enumerate(mapper.edge_segments):
        label = "edge segments" if idx == 0 else None
        ax.plot(
            [segment.start_x_m, segment.end_x_m],
            [segment.start_y_m, segment.end_y_m],
            linewidth=1.4,
            alpha=0.75,
            color="tab:red",
            label=label,
        )

    ax.scatter([0], [0], marker="x", s=70, color="black", label="start pose")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("Fixed-pose LiDAR room map")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=170)
    print(f"saved {save_path}")


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})

    ap = argparse.ArgumentParser(description="Stage 6: accumulate a fixed-pose LiDAR room map")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--save", default="data/captures/lidar_room_map.png")
    ap.add_argument("--point-cloud-csv", default=None)
    ap.add_argument("--edges-csv", default=None)
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--min-points", type=int, default=420, help="LiDAR points to collect per approximate scan")
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
    print("Building a fixed-pose room map. No motors are controlled by this script.")
    print("For now, keep the robot still; odometry can be passed in later when encoders exist.")

    deadline = time.monotonic() + max(0.0, args.seconds)
    try:
        while time.monotonic() <= deadline or mapper.scan_count == 0:
            points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
            points = clean_lidar_points(points, cfg=cfg, filter_config=filter_config, label=f"scan {mapper.scan_count + 1}")
            scan = mapper.add_scan(points)
            print(
                f"scan={mapper.scan_count:03d} points={len(scan.points):04d} "
                f"edges={len(scan.edge_segments):02d} total_points={len(mapper.point_cloud)}"
            )
    except KeyboardInterrupt:
        print("\nstopped; saving current map")
    finally:
        lidar.close()

    if args.point_cloud_csv:
        write_point_cloud_csv(Path(args.point_cloud_csv), mapper.point_cloud)
        print(f"saved {args.point_cloud_csv}")
    if args.edges_csv:
        write_edges_csv(Path(args.edges_csv), mapper.edge_segments)
        print(f"saved {args.edges_csv}")
    plot_room_map(mapper, Path(args.save))


if __name__ == "__main__":
    main()
