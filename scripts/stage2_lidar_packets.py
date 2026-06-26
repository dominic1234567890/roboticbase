#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.perception.lidar_filters import valid_points
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Stage 2: parse LD20 packets")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--skip-crc", action="store_true", help="Decode frames even if CRC check fails")
    args = ap.parse_args()

    lidar = SerialLD20(args.port, args.baud, mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0), check_crc=not args.skip_crc)
    print(f"Reading parsed frames from {args.port} at {args.baud} baud. Ctrl+C to stop.")
    t0 = time.monotonic()
    frames = 0
    points = 0
    try:
        while time.monotonic() - t0 < args.seconds:
            for frame in lidar.read_available_frames():
                frames += 1
                pts = valid_points(frame.points, min_m=cfg["lidar"].get("min_distance_m", 0.08), max_m=cfg["lidar"].get("max_distance_m", 8.0))
                points += len(pts)
                first = pts[0] if pts else None
                if first:
                    print(
                        f"frame={frames:05d} speed={frame.speed_deg_per_s:.0f}deg/s "
                        f"angle={frame.start_angle_deg_cw:.1f}->{frame.end_angle_deg_cw:.1f} "
                        f"valid_pts={len(pts):02d} first={first.distance_m:.3f}m @{first.bearing_deg:+.1f}deg crc={frame.crc_ok}"
                    )
        dt = max(0.001, time.monotonic() - t0)
        print(f"\nSummary: {frames} frames, {points} valid points, {frames/dt:.1f} frames/s")
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
