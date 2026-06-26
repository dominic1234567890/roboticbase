#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import matplotlib.pyplot as plt

from tcr_minibot.perception.lidar_filters import aggregate_one_scan, valid_points
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Stage 3: approximate one 360-degree LD20 scan")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--save", default=None, help="Save plot to path instead of opening a window")
    ap.add_argument("--skip-crc", action="store_true")
    args = ap.parse_args()

    lidar = SerialLD20(args.port, args.baud, mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0), check_crc=not args.skip_crc)
    try:
        print("Collecting one approximate scan...")
        points = aggregate_one_scan(lidar.frames(), min_points=420)
        points = valid_points(points, min_m=cfg["lidar"].get("min_distance_m", 0.08), max_m=cfg["lidar"].get("max_distance_m", 8.0))
    finally:
        lidar.close()

    print(f"Plotting {len(points)} points")
    xs = [p.x_m for p in points]
    ys = [p.y_m for p in points]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(xs, ys, s=3)
    ax.scatter([0], [0], marker="x", s=60)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("LD20 LiDAR scan")
    ax.grid(True)

    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
