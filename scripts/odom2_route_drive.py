#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
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
    parse_route,
    route_summary,
    wrap_pi,
)
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.motion.differential_drive import arcade_to_wheel_power, clamp
from tcr_minibot.odometry.fused_odometry import copy_pose
from tcr_minibot.utils.config import load_config


ARM_FLAG = "--i-understand-this-will-drive-the-robot"


def distance_from(start_x: float, start_y: float, x: float, y: float) -> float:
    return math.hypot(x - start_x, y - start_y)


def stop_and_sleep(motors: DifferentialMotors, seconds: float = 0.25) -> None:
    motors.stop()
    sleep(max(0.0, seconds))


def run_straight_step(reader, odom, motors: DifferentialMotors, args: argparse.Namespace, distance_m: float) -> None:
    start_pose = copy_pose(odom.pose)
    target_heading = start_pose.heading_rad
    target_distance = abs(distance_m)
    direction = 1.0 if distance_m >= 0.0 else -1.0
    stable_since_s: float | None = None
    start_s = monotonic()
    pid = HeadingPidController(heading_pid_config_from_args(args))
    pid.reset(target_heading)

    print(
        f"straight target={distance_m:+.3f} m using heading PID "
        f"kp={args.heading_kp:.3f} ki={args.heading_ki:.3f} kd={args.heading_kd:.3f}"
    )
    while True:
        update = odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=monotonic())
        pose = update.pose
        traveled = distance_from(start_pose.x_m, start_pose.y_m, pose.x_m, pose.y_m)
        remaining = target_distance - traveled

        if remaining <= args.straight_tolerance_m:
            if stable_since_s is None:
                stable_since_s = monotonic()
            if monotonic() - stable_since_s >= args.settle_s:
                stop_and_sleep(motors)
                print(f"straight done: {format_pose(pose.x_m, pose.y_m, pose.heading_rad)}")
                return
        else:
            stable_since_s = None

        if monotonic() - start_s > args.step_timeout_s:
            stop_and_sleep(motors)
            raise TimeoutError("straight step timed out")

        dt_s = update.dt_s or (1.0 / args.control_hz)
        turn_power, heading_error_deg = pid.update(pose.heading_rad, dt_s)
        slow_factor = clamp(remaining / max(args.slowdown_distance_m, 1e-6), 0.35, 1.0)
        forward_power = direction * args.forward_power * slow_factor
        cmd = arcade_to_wheel_power(forward_power, turn_power, max_abs=args.max_power)
        motors.drive_power(cmd)
        print(
            f"straight: remaining={remaining:+.3f} m heading_error={heading_error_deg:+.2f} deg "
            f"turn_correction={turn_power:+.2f}%"
        )
        sleep(1.0 / args.control_hz)


def run_turn_step(reader, odom, motors: DifferentialMotors, args: argparse.Namespace, turn_deg: float) -> None:
    target_heading = wrap_pi(odom.pose.heading_rad + math.radians(turn_deg))
    stable_since_s: float | None = None
    start_s = monotonic()

    print(f"turn target={turn_deg:+.1f} deg")
    while True:
        update = odom.update(reader.read_left_ticks(), reader.read_right_ticks(), now_s=monotonic())
        pose = update.pose
        error_rad = wrap_pi(target_heading - pose.heading_rad)
        error_deg = math.degrees(error_rad)

        if abs(error_deg) <= args.turn_tolerance_deg:
            if stable_since_s is None:
                stable_since_s = monotonic()
            if monotonic() - stable_since_s >= args.settle_s:
                stop_and_sleep(motors)
                print(f"turn done: {format_pose(pose.x_m, pose.y_m, pose.heading_rad)}")
                return
        else:
            stable_since_s = None

        if monotonic() - start_s > args.step_timeout_s:
            stop_and_sleep(motors)
            raise TimeoutError("turn step timed out")

        turn_power = clamp(args.turn_kp * error_deg, -args.turn_power, args.turn_power)
        if abs(turn_power) < args.min_turn_power:
            turn_power = math.copysign(args.min_turn_power, turn_power if turn_power != 0.0 else error_deg)
        cmd = arcade_to_wheel_power(0.0, turn_power, max_abs=args.max_power)
        motors.drive_power(cmd)
        sleep(1.0 / args.control_hz)


def main() -> None:
    cfg = load_config()
    route_cfg = cfg.get("route_drive", {})
    pid_cfg = cfg.get("drive_forward_test", {})
    ap = argparse.ArgumentParser(
        description="Guarded route runner using encoder odometry. Start with wheels off the ground, then low power on the floor."
    )
    ap.add_argument(
        "--route",
        default=route_cfg.get("route", "straight:1.0,right:90,straight:1.0,left:90,straight:1.0"),
        help="Comma route, for example straight:1.0,right:90,straight:1.0,left:90,straight:1.0",
    )
    ap.add_argument("--dry-run", action="store_true", help="Parse/print route but do not touch GPIO or motors")
    ap.add_argument("--enable-motors", action="store_true", help="Required before any motion command is sent")
    ap.add_argument(ARM_FLAG, action="store_true", dest="armed_ack")
    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    add_current_motor_args(ap, cfg)
    control = ap.add_argument_group("route controller")
    control.add_argument("--control-hz", type=float, default=float(route_cfg.get("control_hz", 20.0)))
    control.add_argument("--forward-power", type=float, default=float(route_cfg.get("forward_power_percent", 14.0)))
    control.add_argument("--turn-power", type=float, default=float(route_cfg.get("turn_power_percent", 13.0)))
    control.add_argument("--min-turn-power", type=float, default=float(route_cfg.get("min_turn_power_percent", 7.0)))
    control.add_argument("--turn-kp", type=float, default=float(route_cfg.get("turn_kp", 0.45)), help="Turn power per heading error degree")
    control.add_argument("--slowdown-distance-m", type=float, default=float(route_cfg.get("slowdown_distance_m", 0.25)))
    control.add_argument("--straight-tolerance-m", type=float, default=float(route_cfg.get("straight_tolerance_m", 0.03)))
    control.add_argument("--turn-tolerance-deg", type=float, default=float(route_cfg.get("turn_tolerance_deg", 3.0)))
    control.add_argument("--settle-s", type=float, default=float(route_cfg.get("settle_s", 0.15)))
    control.add_argument("--step-timeout-s", type=float, default=float(route_cfg.get("step_timeout_s", 20.0)))
    add_heading_pid_args(ap, pid_cfg)
    args = ap.parse_args()

    steps = parse_route(args.route)
    print("route:", route_summary(steps))
    print(f"Using motor config: {describe_motor_args(args)}")
    print(
        f"Straight heading PID: kp={args.heading_kp:.3f} ki={args.heading_ki:.3f} "
        f"kd={args.heading_kd:.3f} max_correction={args.max_turn_correction:.1f}%"
    )

    if args.dry_run or not args.enable_motors:
        print("Dry run only. Add --enable-motors and the acknowledgement flag to drive.")
        return
    if not args.armed_ack:
        print(f"Refusing to drive. Re-run with {ARM_FLAG} after the robot is on the floor and the area is clear.")
        raise SystemExit(2)

    reader = make_encoder_reader(args)
    odom = make_odometry(args)
    motors = DifferentialMotors(motor_config_from_args(args), armed=True)

    try:
        odom.reset(
            left_ticks=reader.read_left_ticks(),
            right_ticks=reader.read_right_ticks(),
            now_s=monotonic(),
        )
        stop_and_sleep(motors, 0.5)
        for index, step in enumerate(steps):
            print(f"step {index + 1}/{len(steps)}")
            if step.kind == "straight":
                run_straight_step(reader, odom, motors, args, step.value)
            elif step.kind == "turn":
                run_turn_step(reader, odom, motors, args, step.value)
            else:
                raise ValueError(f"Unhandled route step {step.kind!r}")
        final = odom.pose
        print(f"route complete: {format_pose(final.x_m, final.y_m, final.heading_rad)}")
    except KeyboardInterrupt:
        print("\nRoute interrupted; stopping motors.")
    finally:
        motors.stop()
        reader.close()


if __name__ == "__main__":
    main()
