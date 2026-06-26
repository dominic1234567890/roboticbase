#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, EdgeSegment, extract_edge_segments
from tcr_minibot.perception.lidar_filters import aggregate_one_scan
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.utils.config import load_config


def write_segments_csv(path: Path, segments: list[EdgeSegment]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "start_x_m",
                "start_y_m",
                "end_x_m",
                "end_y_m",
                "length_m",
                "heading_deg",
                "point_count",
                "rms_error_m",
            ]
        )
        for segment in segments:
            writer.writerow(
                [
                    f"{segment.start_x_m:.4f}",
                    f"{segment.start_y_m:.4f}",
                    f"{segment.end_x_m:.4f}",
                    f"{segment.end_y_m:.4f}",
                    f"{segment.length_m:.4f}",
                    f"{segment.heading_deg:.2f}",
                    segment.point_count,
                    f"{segment.rms_error_m:.4f}",
                ]
            )


def plot_edge_scan(points: list[LidarPoint], segments: list[EdgeSegment], save_path: Path | None) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter([p.x_m for p in points], [p.y_m for p in points], s=3, alpha=0.45, label="LiDAR points")
    ax.scatter([0], [0], marker="x", s=70, color="black", label="LiDAR")

    for idx, segment in enumerate(segments):
        label = "edge segments" if idx == 0 else None
        ax.plot(
            [segment.start_x_m, segment.end_x_m],
            [segment.start_y_m, segment.end_y_m],
            linewidth=2.2,
            color="tab:red",
            label=label,
        )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("LD20 edge map")
    ax.grid(True)
    ax.legend(loc="upper right")

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=160)
        print(f"saved {save_path}")
    else:
        plt.show()


def main() -> None:
    cfg = load_config()
    mapping_cfg = cfg.get("mapping", {})
    ap = argparse.ArgumentParser(description="Stage 5: extract wall-like edge segments from one LiDAR scan")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--save", default=None, help="Save plot to path instead of opening a window")
    ap.add_argument("--csv", default=None, help="Optional CSV path for extracted edge segments")
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--min-points", type=int, default=420, help="LiDAR points to collect for one approximate scan")
    ap.add_argument(
        "--max-gap-m",
        type=float,
        default=mapping_cfg.get("edge_max_neighbor_gap_m", 0.18),
        help="Split scan clusters across larger point gaps",
    )
    ap.add_argument(
        "--max-line-error-m",
        type=float,
        default=mapping_cfg.get("edge_max_line_error_m", 0.035),
        help="Split segments above this line-fit error",
    )
    ap.add_argument("--min-segment-points", type=int, default=mapping_cfg.get("edge_min_segment_points", 8))
    ap.add_argument("--min-segment-length-m", type=float, default=mapping_cfg.get("edge_min_segment_length_m", 0.25))
    add_lidar_filter_args(ap, mapping_cfg)
    args = ap.parse_args()
    filter_config = filter_config_from_args(args)

    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )
    try:
        print("Collecting one approximate scan. No motors are controlled by this script.")
        points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
        points = clean_lidar_points(points, cfg=cfg, filter_config=filter_config, label="scan")
    finally:
        lidar.close()

    edge_config = EdgeMappingConfig(
        max_neighbor_gap_m=args.max_gap_m,
        max_line_error_m=args.max_line_error_m,
        min_points_per_segment=args.min_segment_points,
        min_segment_length_m=args.min_segment_length_m,
    )
    segments = extract_edge_segments(points, edge_config)
    print(f"points={len(points)} edge_segments={len(segments)}")
    for idx, segment in enumerate(segments[:12], start=1):
        mid_x, mid_y = segment.midpoint_m
        print(
            f"{idx:02d}: len={segment.length_m:.2f}m heading={segment.heading_deg:6.1f}deg "
            f"mid=({mid_x:.2f}, {mid_y:.2f}) points={segment.point_count} rms={segment.rms_error_m:.3f}m"
        )

    if args.csv:
        write_segments_csv(Path(args.csv), segments)
        print(f"saved {args.csv}")

    plot_edge_scan(points, segments, Path(args.save) if args.save else None)


if __name__ == "__main__":
    main()
