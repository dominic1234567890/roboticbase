#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import add_current_motor_args
from tcr_minibot.hardware.motors import DifferentialMotors, MotorConfig
from tcr_minibot.motion.differential_drive import WheelCommands
from tcr_minibot.perception.lidar_filters import aggregate_one_scan, compute_zone_distances, valid_points
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.utils.config import load_config


ARM_FLAG = "--i-understand-this-can-move-the-robot"


def front_is_blocked(lidar: SerialLD20, args: argparse.Namespace, cfg: dict, *, label: str) -> tuple[bool, float | None]:
    raw_points = aggregate_one_scan(lidar.frames(), min_points=args.min_points)
    points = valid_points(
        raw_points,
        min_m=cfg["lidar"].get("min_distance_m", 0.08),
        max_m=cfg["lidar"].get("max_distance_m", 8.0),
        min_confidence=cfg["lidar"].get("min_confidence", 0),
    )
    zones = compute_zone_distances(points, front_half_angle_deg=args.front_half_angle_deg)
    distance = zones.front_min_m
    blocked = distance is None or distance < args.front_stop_distance_m
    state = "BLOCKED" if blocked else "CLEAR"
    shown_distance = "unknown" if distance is None else f"{distance:.3f} m"
    print(f"{label}: front={shown_distance} threshold={args.front_stop_distance_m:.3f} m -> {state}")
    return blocked, distance


def main() -> None:
    cfg = load_config()
    safety_cfg = cfg.get("safety", {})

    ap = argparse.ArgumentParser(
        description="LiDAR front-zone motion gate. Refuses forward motor motion when the front zone is blocked."
    )
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--min-points", type=int, default=300)
    ap.add_argument("--front-stop-distance-m", type=float, default=float(safety_cfg.get("front_stop_distance_m", 0.35)))
    ap.add_argument("--front-half-angle-deg", type=float, default=float(safety_cfg.get("front_zone_half_angle_deg", 25.0)))
    ap.add_argument("--drive-forward-test", action="store_true", help="Actually try a short forward motor move if clear")
    ap.add_argument("--power", type=float, default=12.0, help="Forward test power percent")
    ap.add_argument("--seconds", type=float, default=1.0, help="Forward test duration")
    ap.add_argument("--poll-s", type=float, default=0.20, help="Minimum delay between safety checks while moving")
    ap.add_argument(ARM_FLAG, action="store_true", dest="armed_ack")
    add_current_motor_args(ap)
    args = ap.parse_args()

    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )

    motors: DifferentialMotors | None = None
    try:
        blocked, _ = front_is_blocked(lidar, args, cfg, label="precheck")
        if not args.drive_forward_test:
            if blocked:
                print("Motion would be refused. Not driving motors because --drive-forward-test was not set.")
                raise SystemExit(2)
            print("Front zone is clear. Not driving motors because --drive-forward-test was not set.")
            return

        if not args.armed_ack:
            print(f"Refusing to drive. Re-run with {ARM_FLAG} after the robot is on the floor and the space is clear.")
            raise SystemExit(2)

        motor_cfg = MotorConfig(
            left_port=args.left_motor_port,
            right_port=args.right_motor_port,
            left_reversed=args.left_motor_reversed,
            right_reversed=args.right_motor_reversed,
            max_power_percent=args.max_power,
        )
        motors = DifferentialMotors(motor_cfg, armed=True)
        if blocked:
            motors.stop()
            print("Emergency stop gate refused motion because the front zone is blocked or unknown.")
            raise SystemExit(2)

        print("Front is clear. Starting guarded forward test.")
        end_s = monotonic() + max(0.0, args.seconds)
        while monotonic() < end_s:
            blocked, _ = front_is_blocked(lidar, args, cfg, label="moving-check")
            if blocked:
                motors.stop()
                print("Emergency stop: front zone became blocked.")
                raise SystemExit(2)
            motors.drive_power(WheelCommands(left=args.power, right=args.power))
            sleep(max(0.02, args.poll_s))
        motors.stop()
        print("Guarded forward test complete; motors stopped.")
    finally:
        if motors is not None:
            motors.stop()
        lidar.close()


if __name__ == "__main__":
    main()
