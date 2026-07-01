#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import add_encoder_pin_args, add_odometry_args, format_pose, make_encoder_reader, make_odometry
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Compute differential-drive pose from wheel encoders only")
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    ap.add_argument("--print-hz", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means forever.")
    args = ap.parse_args()

    period_s = 1.0 / max(0.1, args.print_hz)
    reader = make_encoder_reader(args)
    odom = make_odometry(args)

    left0 = reader.read_left_ticks()
    right0 = reader.read_right_ticks()
    odom.reset(left_ticks=left0, right_ticks=right0, now_s=monotonic())

    print("Wheel odometry watch started.")
    print("Push the robot by hand or run motors from another terminal at very low power.")
    print("If forward motion makes x go negative, change left_inverted/right_inverted in config/robot.yaml or override with CLI flags.")
    print(f"encoder signs: left_invert={args.left_invert} right_invert={args.right_invert}")
    print(
        f"calibration: wheel_radius={args.wheel_radius_m:.4f} m, "
        f"wheel_track={args.wheel_track_m:.4f} m, ticks_per_rev={args.ticks_per_rev}"
    )

    start_s = monotonic()
    try:
        while args.duration <= 0.0 or monotonic() - start_s < args.duration:
            sleep(period_s)
            update = odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=monotonic())
            pose = update.pose
            print(
                f"{format_pose(pose.x_m, pose.y_m, pose.heading_rad)}  "
                f"dL={update.left_distance_m:+.4f} m dR={update.right_distance_m:+.4f} m "
                f"dtheta={math.degrees(update.used_heading_delta_rad):+.2f} deg source={update.heading_source}"
            )
    except KeyboardInterrupt:
        print("\nStopping odometry watch.")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
