#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.perception.lidar_filters import aggregate_one_scan, compute_zone_distances, valid_points
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.utils.config import load_config


def fmt(v: float | None) -> str:
    return "----" if v is None else f"{v:0.2f}m"


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Stage 4: no-motor LiDAR safety bubble")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    args = ap.parse_args()

    safety = cfg["safety"]
    lidar = SerialLD20(args.port, args.baud, mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0), check_crc=not args.skip_crc)
    print("No motors are controlled by this script. Move your hand around the LiDAR and watch distances change.")
    try:
        while True:
            pts = aggregate_one_scan(lidar.frames(), min_points=360)
            pts = valid_points(pts, min_m=cfg["lidar"].get("min_distance_m", 0.08), max_m=cfg["lidar"].get("max_distance_m", 8.0))
            zones = compute_zone_distances(
                pts,
                front_half_angle_deg=safety.get("front_zone_half_angle_deg", 25),
                side_half_angle_deg=safety.get("side_zone_width_deg", 35) / 2,
            )
            blocked = zones.front_min_m is not None and zones.front_min_m < safety.get("front_stop_distance_m", 0.35)
            print(
                f"front={fmt(zones.front_min_m)} left={fmt(zones.left_min_m)} "
                f"right={fmt(zones.right_min_m)} rear={fmt(zones.rear_min_m)} | "
                f"front_blocked={'YES' if blocked else 'no '}"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
