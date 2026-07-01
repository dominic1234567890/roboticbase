#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import (
    HeadingPidController,
    add_current_motor_args,
    add_encoder_pin_args,
    add_heading_pid_args,
    add_odometry_args,
    describe_motor_args,
    format_pose,
    heading_pid_config_from_args,
    make_encoder_reader,
    make_odometry,
    motor_config_from_args,
)
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.motion.differential_drive import WheelCommands, arcade_to_wheel_power
from tcr_minibot.perception.lidar_filters import aggregate_one_scan, compute_zone_distances, valid_points
from tcr_minibot.sensors.lidar_ld20 import SerialLD20
from tcr_minibot.utils.config import load_config


ARM_FLAG = "--i-understand-this-can-move-the-robot"
DRIVE_MODES = ("open-loop", "heading-pid")


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
    pid_cfg = cfg.get("drive_forward_test", {})

    ap = argparse.ArgumentParser(
        description="LiDAR front-zone motion gate. Refuses forward motor motion when the front zone is blocked."
    )
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--min-points", type=int, default=int(safety_cfg.get("front_gate_min_points", 900)))
    ap.add_argument("--front-stop-distance-m", type=float, default=float(safety_cfg.get("front_stop_distance_m", 0.35)))
    ap.add_argument("--front-half-angle-deg", type=float, default=float(safety_cfg.get("front_zone_half_angle_deg", 25.0)))
    ap.add_argument("--drive-forward-test", action="store_true", help="Actually try a short forward motor move if clear")
    ap.add_argument("--drive-mode", choices=DRIVE_MODES, default=safety_cfg.get("front_gate_drive_mode", "heading-pid"))
    ap.add_argument("--power", type=float, default=float(safety_cfg.get("front_gate_test_power_percent", 12.0)), help="Forward test power percent")
    ap.add_argument("--seconds", type=float, default=float(safety_cfg.get("front_gate_test_seconds", 1.0)), help="Forward test duration")
    ap.add_argument("--poll-s", type=float, default=float(safety_cfg.get("front_gate_poll_s", 0.20)), help="Minimum delay between safety checks while moving")
    ap.add_argument(ARM_FLAG, action="store_true", dest="armed_ack")
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    add_heading_pid_args(ap, pid_cfg)
    add_current_motor_args(ap, cfg)
    args = ap.parse_args()

    print(f"Using motor config: {describe_motor_args(args)}")
    print(
        f"Using safety gate: min_points={args.min_points} "
        f"front_half_angle={args.front_half_angle_deg:.1f} deg "
        f"stop_distance={args.front_stop_distance_m:.3f} m drive_mode={args.drive_mode}"
    )
    if args.drive_mode == "heading-pid":
        print(
            f"Straight heading PID: kp={args.heading_kp:.3f} ki={args.heading_ki:.3f} "
            f"kd={args.heading_kd:.3f} max_correction={args.max_turn_correction:.1f}%"
        )

    lidar = SerialLD20(
        args.port,
        args.baud,
        mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0),
        check_crc=not args.skip_crc,
    )

    motors: DifferentialMotors | None = None
    reader = None
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

        motors = DifferentialMotors(motor_config_from_args(args), armed=True)
        if blocked:
            motors.stop()
            print("Emergency stop gate refused motion because the front zone is blocked or unknown.")
            raise SystemExit(2)

        odom = None
        pid = None
        if args.drive_mode == "heading-pid":
            reader = make_encoder_reader(args)
            reader.reset()
            odom = make_odometry(args)
            odom.reset(
                left_ticks=reader.read_left_ticks(),
                right_ticks=reader.read_right_ticks(),
                now_s=monotonic(),
            )
            pid = HeadingPidController(heading_pid_config_from_args(args))
            pid.reset(odom.pose.heading_rad)

        print("Front is clear. Starting guarded forward test.")
        end_s = monotonic() + max(0.0, args.seconds)
        while monotonic() < end_s:
            blocked, _ = front_is_blocked(lidar, args, cfg, label="moving-check")
            if blocked:
                motors.stop()
                print("Emergency stop: front zone became blocked.")
                raise SystemExit(2)

            if args.drive_mode == "heading-pid" and reader is not None and odom is not None and pid is not None:
                update = odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=monotonic())
                turn_correction, heading_error_deg = pid.update(update.pose.heading_rad, update.dt_s or args.poll_s)
                cmd = arcade_to_wheel_power(args.power, turn_correction, max_abs=args.max_power)
                motors.drive_power(cmd)
                print(
                    f"heading-pid: {format_pose(update.pose.x_m, update.pose.y_m, update.pose.heading_rad)} "
                    f"err={heading_error_deg:+.2f} deg turn={turn_correction:+.2f}%"
                )
            else:
                motors.drive_power(WheelCommands(left=args.power, right=args.power))
            sleep(max(0.02, args.poll_s))
        motors.stop()
        print("Guarded forward test complete; motors stopped.")
    finally:
        if motors is not None:
            motors.stop()
        if reader is not None:
            reader.close()
        lidar.close()


if __name__ == "__main__":
    main()
