#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from _odom_common import add_encoder_pin_args, add_odometry_args, format_pose, make_encoder_reader, make_odometry
from tcr_minibot.odometry.differential_odometry import Pose2D
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, RoomMapper
from tcr_minibot.perception.occupancy_grid import OccupancyGrid
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.utils.config import load_config


def collect_scan_with_odom(
    lidar: SerialLD20,
    reader,
    odom,
    *,
    min_points: int,
    deadline_s: float,
) -> tuple[list[LidarPoint], Pose2D]:
    points: list[LidarPoint] = []
    last_update_s = monotonic()
    while len(points) < min_points and monotonic() < deadline_s:
        for frame in lidar.read_available_frames():
            points.extend(frame.points)
        now_s = monotonic()
        if now_s - last_update_s >= 0.01:
            odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=now_s)
            last_update_s = now_s
        sleep(0.002)
    update = odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=monotonic())
    return points, update.pose


def write_pose_csv(path: Path, rows: list[tuple[int, float, Pose2D, int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_index", "time_s", "x_m", "y_m", "heading_deg", "filtered_points"])
        for scan_index, time_s, pose, point_count in rows:
            writer.writerow([
                scan_index,
                f"{time_s:.3f}",
                f"{pose.x_m:.4f}",
                f"{pose.y_m:.4f}",
                f"{math.degrees(pose.heading_rad):.2f}",
                point_count,
            ])


def write_point_cloud_csv(path: Path, mapper: RoomMapper) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x_m", "y_m"])
        for point in mapper.point_cloud:
            writer.writerow([f"{point.x_m:.4f}", f"{point.y_m:.4f}"])


def plot_map(mapper: RoomMapper, poses: list[Pose2D], save_path: Path) -> None:
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
        ax.scatter([p.x_m for p in mapper.point_cloud], [p.y_m for p in mapper.point_cloud], s=2, alpha=0.25, label="LiDAR points")

    for segment in mapper.edge_segments:
        ax.plot(
            [segment.start_x_m, segment.end_x_m],
            [segment.start_y_m, segment.end_y_m],
            linewidth=1.0,
            alpha=0.55,
        )

    if poses:
        xs = [pose.x_m for pose in poses]
        ys = [pose.y_m for pose in poses]
        ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.2, label="odometry path")
        final = poses[-1]
        ax.arrow(
            final.x_m,
            final.y_m,
            0.25 * math.cos(final.heading_rad),
            0.25 * math.sin(final.heading_rad),
            head_width=0.05,
            length_includes_head=True,
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward from start (m)")
    ax.set_ylabel("y left from start (m)")
    ax.set_title("Stage 8: LiDAR accumulated with encoder odometry")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=170)
    print(f"saved {save_path}")


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})

    ap = argparse.ArgumentParser(
        description="Stage 8: accumulate horizontal LD20 scans while the robot moves, using encoder odometry"
    )
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--min-points", type=int, default=360, help="LiDAR points per mini-scan")
    ap.add_argument("--save", default="data/captures/lidar_odom_map.png")
    ap.add_argument("--pose-csv", default="data/captures/lidar_odom_poses.csv")
    ap.add_argument("--point-cloud-csv", default=None)
    ap.add_argument("--max-gap-m", type=float, default=mapping_cfg.get("edge_max_neighbor_gap_m", 0.18))
    ap.add_argument("--max-line-error-m", type=float, default=mapping_cfg.get("edge_max_line_error_m", 0.035))
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
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

    reader = make_encoder_reader(args)
    odom = make_odometry(args)
    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )

    left0 = reader.read_left_ticks()
    right0 = reader.read_right_ticks()
    odom.reset(left_ticks=left0, right_ticks=right0, now_s=monotonic())

    print("This script does not drive motors. Push the robot by hand or drive it from another terminal.")
    print("It assumes the LD20 is still horizontal and builds a 2D room slice from moving scans.")
    print("For best first results: move slowly in straight lines and keep ticks_per_rev calibrated.")

    poses: list[Pose2D] = []
    pose_rows: list[tuple[int, float, Pose2D, int]] = []
    start_s = monotonic()
    deadline_s = start_s + max(0.0, args.seconds)
    scan_index = 0

    try:
        while monotonic() < deadline_s:
            raw_points, pose = collect_scan_with_odom(
                lidar,
                reader,
                odom,
                min_points=args.min_points,
                deadline_s=deadline_s,
            )
            if not raw_points:
                break
            filtered_points = clean_lidar_points(
                raw_points,
                cfg=cfg,
                filter_config=filter_config,
                label=f"scan {scan_index}",
            )
            mapper.add_scan(filtered_points, pose=pose)
            poses.append(pose)
            elapsed_s = monotonic() - start_s
            pose_rows.append((scan_index, elapsed_s, pose, len(filtered_points)))
            print(f"scan={scan_index:03d} {format_pose(pose.x_m, pose.y_m, pose.heading_rad)} filtered={len(filtered_points)}")
            scan_index += 1
    except KeyboardInterrupt:
        print("\nStopping moving mapper early.")
    finally:
        lidar.close()
        reader.close()

    if args.pose_csv:
        write_pose_csv(Path(args.pose_csv), pose_rows)
        print(f"saved {args.pose_csv}")
    if args.point_cloud_csv:
        write_point_cloud_csv(Path(args.point_cloud_csv), mapper)
        print(f"saved {args.point_cloud_csv}")

    plot_map(mapper, poses, Path(args.save))


if __name__ == "__main__":
    main()
