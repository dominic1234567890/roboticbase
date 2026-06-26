#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import statistics
import sys
import time
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _lidar_filter_cli import add_lidar_filter_args, clean_lidar_points, filter_config_from_args
from tcr_minibot.perception.lidar_filters import PointCloudFilterConfig, aggregate_one_scan
from tcr_minibot.perception.point_cloud_export import (
    PointCloudRow,
    rows_from_scans,
    write_cloudcompare_ply,
    write_points_csv,
    write_xyz,
)
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20


DEFAULT_CFG: dict[str, Any] = {
    "lidar": {
        "port": "/dev/ttyUSB0",
        "baud": 230400,
        "mount_yaw_offset_deg": 0.0,
        "min_distance_m": 0.08,
        "max_distance_m": 8.0,
        "min_confidence": 0,
    },
    "mapping": {
        "size_m": 6.0,
    },
}


def load_config_or_defaults() -> dict[str, Any]:
    try:
        from tcr_minibot.utils.config import load_config

        return load_config()
    except ModuleNotFoundError as exc:
        if exc.name == "yaml":
            return DEFAULT_CFG
        raise


def parse_formats(value: str) -> set[str]:
    formats = {part.strip().lower() for part in value.split(",") if part.strip()}
    if "none" in formats:
        return set()
    allowed = {"png", "ply", "xyz", "csv"}
    unknown = formats - allowed
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown format(s): {', '.join(sorted(unknown))}")
    return formats


def timestamped_prefix(prefix: str, *, add_timestamp: bool) -> Path:
    path = Path(prefix)
    if not add_timestamp:
        return path
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return path.with_name(f"{path.name}_{stamp}")


def collect_scan(
    lidar: SerialLD20,
    *,
    args: argparse.Namespace,
    cfg: dict[str, Any],
    filter_config: PointCloudFilterConfig,
    scan_index: int,
) -> list[LidarPoint]:
    points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
    filtered = clean_lidar_points(points, cfg=cfg, filter_config=filter_config, label=f"scan {scan_index}")
    print_scan_summary(filtered, label=f"scan {scan_index}")
    return filtered


def print_scan_summary(points: list[LidarPoint], *, label: str) -> None:
    if not points:
        print(f"{label}: no valid points")
        return

    ranges = [p.distance_m for p in points]
    confidences = [p.confidence for p in points]
    print(
        f"{label}: range min/median/max="
        f"{min(ranges):.3f}/{statistics.median(ranges):.3f}/{max(ranges):.3f}m "
        f"confidence min/median/max={min(confidences)}/{statistics.median(confidences):.0f}/{max(confidences)}"
    )


def write_exports(rows: list[PointCloudRow], *, prefix: Path, formats: set[str]) -> list[Path]:
    written: list[Path] = []
    if "ply" in formats:
        path = prefix.with_suffix(".ply")
        write_cloudcompare_ply(path, rows)
        written.append(path)
    if "xyz" in formats:
        path = prefix.with_suffix(".xyz")
        write_xyz(path, rows)
        written.append(path)
    if "csv" in formats:
        path = prefix.with_suffix(".csv")
        write_points_csv(path, rows)
        written.append(path)
    return written


def plot_scans(
    scans: list[list[LidarPoint]],
    *,
    save_path: Path | None,
    show: bool,
    view_size_m: float,
) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 7))
    for idx, points in enumerate(scans, start=1):
        if not points:
            continue
        ax.scatter(
            [p.x_m for p in points],
            [p.y_m for p in points],
            s=3,
            alpha=0.55,
            label=f"scan {idx}",
        )

    half = view_size_m / 2.0
    ax.scatter([0], [0], marker="x", s=70, color="black", label="LiDAR")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("LD20 LiDAR horizontal point slice")
    ax.grid(True)
    ax.legend(loc="upper right")

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=170)
        print(f"saved {save_path}")
    if show:
        plt.show()
    else:
        plt.close(fig)


def live_view(
    lidar: SerialLD20,
    *,
    args: argparse.Namespace,
    cfg: dict[str, Any],
    filter_config: PointCloudFilterConfig,
) -> None:
    import matplotlib.pyplot as plt

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    scatter = ax.scatter([], [], s=4)
    half = args.view_size_m / 2.0
    ax.scatter([0], [0], marker="x", s=70, color="black")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("Live LD20 LiDAR points")
    ax.grid(True)

    scan_index = 1
    try:
        while True:
            points = collect_scan(lidar, args=args, cfg=cfg, filter_config=filter_config, scan_index=scan_index)
            scatter.set_offsets([(p.x_m, p.y_m) for p in points])
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(args.live_delay_s)
            scan_index += 1
    except KeyboardInterrupt:
        print("\nstopped live view")
    finally:
        plt.ioff()


def main() -> None:
    cfg = load_config_or_defaults()
    mapping_cfg = cfg.get("mapping", {})

    ap = argparse.ArgumentParser(description="Capture, view, and export LD20 LiDAR scans for calibration")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--scans", type=int, default=1, help="Number of approximate 360-degree scans to capture")
    ap.add_argument("--min-points", type=int, default=420, help="LiDAR points to collect per approximate scan")
    ap.add_argument("--out-prefix", default="data/captures/lidar_calibration", help="Output path without extension")
    ap.add_argument("--no-timestamp", action="store_true", help="Write exactly to --out-prefix without adding a timestamp")
    ap.add_argument(
        "--formats",
        type=parse_formats,
        default=parse_formats("png,ply,xyz,csv"),
        help="Comma-separated export formats: png,ply,xyz,csv, or none",
    )
    ap.add_argument("--show", action="store_true", help="Open a static 2D plot after capture")
    ap.add_argument("--live", action="store_true", help="Continuously show a 2D point plot until Ctrl+C")
    ap.add_argument("--live-delay-s", type=float, default=0.05)
    ap.add_argument("--view-size-m", type=float, default=float(mapping_cfg.get("size_m", 6.0)))
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
        if args.live:
            print("Starting live 2D LiDAR view. Press Ctrl+C to stop.")
            live_view(lidar, args=args, cfg=cfg, filter_config=filter_config)
            return

        scans: list[list[LidarPoint]] = []
        for scan_index in range(1, max(1, args.scans) + 1):
            scans.append(collect_scan(lidar, args=args, cfg=cfg, filter_config=filter_config, scan_index=scan_index))
    finally:
        lidar.close()

    rows = rows_from_scans((idx, points) for idx, points in enumerate(scans, start=1))
    prefix = timestamped_prefix(args.out_prefix, add_timestamp=not args.no_timestamp)
    written = write_exports(rows, prefix=prefix, formats=args.formats)
    if "png" in args.formats:
        png_path = prefix.with_suffix(".png")
        plot_scans(scans, save_path=png_path, show=args.show, view_size_m=args.view_size_m)
        written.append(png_path)
    elif args.show:
        plot_scans(scans, save_path=None, show=True, view_size_m=args.view_size_m)

    if written:
        print("wrote:")
        for path in written:
            print(f"  {path}")


if __name__ == "__main__":
    main()
