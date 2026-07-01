#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import (
    add_current_motor_args,
    add_encoder_pin_args,
    add_odometry_args,
    describe_motor_args,
    format_pose,
    make_encoder_reader,
    make_odometry,
    motor_config_from_args,
)
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.motion.differential_drive import WheelCommands, arcade_to_wheel_power, clamp
from tcr_minibot.odometry.fused_odometry import wrap_pi
from tcr_minibot.utils.config import load_config


ARM_FLAG = "--i-understand-this-will-drive-the-robot"
MODES = ("open-loop", "heading-pid")


def _cfg_float(cfg: dict, key: str, default: float) -> float:
    return float(cfg.get(key, default))


def _cfg_str(cfg: dict, key: str, default: str) -> str:
    value = cfg.get(key, default)
    return default if value is None else str(value)


def _csv_enabled(path_text: str | None) -> bool:
    if path_text is None:
        return False
    return path_text.strip().lower() not in {"", "none", "null", "false", "off"}


def write_csv(path_text: str | None, rows: list[dict[str, object]]) -> None:
    if not _csv_enabled(path_text):
        return
    path = Path(str(path_text))
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "time_s",
        "left_ticks",
        "right_ticks",
        "x_m",
        "y_m",
        "distance_from_start_m",
        "heading_deg",
        "heading_error_deg",
        "turn_correction_percent",
        "left_command_percent",
        "right_command_percent",
        "left_delta_m",
        "right_delta_m",
        "center_delta_m",
        "dt_s",
        "mode",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved {path}")


def distance_from_origin(x_m: float, y_m: float) -> float:
    return math.hypot(x_m, y_m)


def main() -> None:
    cfg = load_config()
    test_cfg = cfg.get("drive_forward_test", {})

    ap = argparse.ArgumentParser(
        description=(
            "Drive straight test using motor config from robot.yaml. "
            "Start with mode=open-loop to measure drift, then try mode=heading-pid for encoder heading correction."
        )
    )
    ap.add_argument("--dry-run", action="store_true", help="Print resolved config but do not touch GPIO or motors")
    ap.add_argument("--enable-motors", action="store_true", help="Required before any motion command is sent")
    ap.add_argument(ARM_FLAG, action="store_true", dest="armed_ack")

    control = ap.add_argument_group("drive command")
    control.add_argument("--mode", choices=MODES, default=_cfg_str(test_cfg, "mode", "open-loop"))
    control.add_argument("--power", type=float, default=_cfg_float(test_cfg, "power_percent", 12.0))
    control.add_argument("--seconds", type=float, default=_cfg_float(test_cfg, "seconds", 1.0))
    control.add_argument(
        "--target-distance-m",
        type=float,
        default=_cfg_float(test_cfg, "target_distance_m", 0.0),
        help="Optional odometry distance target. 0 disables distance stop and uses --seconds only.",
    )
    control.add_argument("--control-hz", type=float, default=_cfg_float(test_cfg, "control_hz", 20.0))
    control.add_argument("--print-hz", type=float, default=_cfg_float(test_cfg, "print_hz", 5.0))
    control.add_argument("--left-trim", type=float, default=_cfg_float(test_cfg, "left_trim_percent", 0.0))
    control.add_argument("--right-trim", type=float, default=_cfg_float(test_cfg, "right_trim_percent", 0.0))
    control.add_argument("--csv", default=_cfg_str(test_cfg, "csv", "data/captures/drive_forward_test.csv"))

    pid = ap.add_argument_group("heading PID; only used with --mode heading-pid")
    pid.add_argument("--heading-kp", type=float, default=_cfg_float(test_cfg, "heading_kp", 0.45))
    pid.add_argument("--heading-ki", type=float, default=_cfg_float(test_cfg, "heading_ki", 0.0))
    pid.add_argument("--heading-kd", type=float, default=_cfg_float(test_cfg, "heading_kd", 0.0))
    pid.add_argument(
        "--integral-limit-deg-s",
        type=float,
        default=_cfg_float(test_cfg, "integral_limit_deg_s", 60.0),
        help="Anti-windup clamp for heading error integral.",
    )
    pid.add_argument(
        "--max-turn-correction",
        type=float,
        default=_cfg_float(test_cfg, "max_turn_correction_percent", 8.0),
        help="Maximum left/right steering correction in motor power percent.",
    )

    add_encoder_pin_args(ap, cfg)
    add_odometry_args(ap, cfg)
    add_current_motor_args(ap, cfg)
    args = ap.parse_args()

    print(f"Using motor config: {describe_motor_args(args)}")
    print(
        f"Using encoder pins: LEFT=GPIO{args.left_a}/GPIO{args.left_b} "
        f"RIGHT=GPIO{args.right_a}/GPIO{args.right_b} "
        f"invert left={args.left_invert} right={args.right_invert}"
    )
    print(
        f"drive_forward_test: mode={args.mode} power={args.power:.1f}% "
        f"seconds={args.seconds:.2f} target_distance={args.target_distance_m:.3f} m "
        f"trim L={args.left_trim:+.2f}% R={args.right_trim:+.2f}%"
    )
    if args.mode == "heading-pid":
        print(
            f"heading PID: kp={args.heading_kp:.3f} ki={args.heading_ki:.3f} kd={args.heading_kd:.3f} "
            f"max_turn_correction={args.max_turn_correction:.1f}%"
        )

    if args.dry_run or not args.enable_motors:
        print("Dry run only. Add --enable-motors and the acknowledgement flag to drive.")
        return
    if not args.armed_ack:
        print(f"Refusing to drive. Re-run with {ARM_FLAG} after the robot is on the floor and the area is clear.")
        raise SystemExit(2)

    if args.control_hz <= 0.0:
        raise ValueError("--control-hz must be greater than zero")
    if args.print_hz <= 0.0:
        raise ValueError("--print-hz must be greater than zero")

    rows: list[dict[str, object]] = []
    reader = None
    motors: DifferentialMotors | None = None

    try:
        reader = make_encoder_reader(args)
        reader.reset()
        odom = make_odometry(args)
        start_s = monotonic()
        odom.reset(
            left_ticks=reader.read_left_ticks(),
            right_ticks=reader.read_right_ticks(),
            now_s=start_s,
        )
        target_heading_rad = odom.pose.heading_rad

        motors = DifferentialMotors(motor_config_from_args(args), armed=True)
        period_s = 1.0 / args.control_hz
        print_period_s = 1.0 / args.print_hz
        next_print_s = start_s
        previous_error_deg: float | None = None
        integral_deg_s = 0.0

        print("Starting drive-forward test.")
        while True:
            loop_start_s = monotonic()
            elapsed_s = loop_start_s - start_s
            left_ticks = reader.read_left_ticks()
            right_ticks = reader.read_right_ticks()
            update = odom.update(left_ticks, right_ticks, now_s=loop_start_s)
            pose = update.pose
            traveled_m = distance_from_origin(pose.x_m, pose.y_m)

            if elapsed_s >= max(0.0, args.seconds):
                break
            if args.target_distance_m > 0.0 and traveled_m >= args.target_distance_m:
                break

            heading_error_deg = math.degrees(wrap_pi(target_heading_rad - pose.heading_rad))
            dt_s = max(1e-6, update.dt_s or period_s)
            derivative_deg_s = 0.0 if previous_error_deg is None else (heading_error_deg - previous_error_deg) / dt_s
            previous_error_deg = heading_error_deg

            turn_correction = 0.0
            if args.mode == "heading-pid":
                integral_deg_s += heading_error_deg * dt_s
                integral_limit = abs(args.integral_limit_deg_s)
                integral_deg_s = clamp(integral_deg_s, -integral_limit, integral_limit)
                turn_correction = (
                    args.heading_kp * heading_error_deg
                    + args.heading_ki * integral_deg_s
                    + args.heading_kd * derivative_deg_s
                )
                turn_correction = clamp(turn_correction, -abs(args.max_turn_correction), abs(args.max_turn_correction))
            else:
                integral_deg_s = 0.0

            cmd = arcade_to_wheel_power(args.power, turn_correction, max_abs=args.max_power)
            left_command = clamp(cmd.left + args.left_trim, -args.max_power, args.max_power)
            right_command = clamp(cmd.right + args.right_trim, -args.max_power, args.max_power)
            motors.drive_power(WheelCommands(left=left_command, right=right_command))

            rows.append(
                {
                    "time_s": f"{elapsed_s:.4f}",
                    "left_ticks": left_ticks,
                    "right_ticks": right_ticks,
                    "x_m": f"{pose.x_m:.5f}",
                    "y_m": f"{pose.y_m:.5f}",
                    "distance_from_start_m": f"{traveled_m:.5f}",
                    "heading_deg": f"{math.degrees(pose.heading_rad):.3f}",
                    "heading_error_deg": f"{heading_error_deg:.3f}",
                    "turn_correction_percent": f"{turn_correction:.3f}",
                    "left_command_percent": f"{left_command:.3f}",
                    "right_command_percent": f"{right_command:.3f}",
                    "left_delta_m": f"{update.left_distance_m:.5f}",
                    "right_delta_m": f"{update.right_distance_m:.5f}",
                    "center_delta_m": f"{update.center_distance_m:.5f}",
                    "dt_s": "" if update.dt_s is None else f"{update.dt_s:.5f}",
                    "mode": args.mode,
                }
            )

            if loop_start_s >= next_print_s:
                print(
                    f"t={elapsed_s:5.2f}s dist={traveled_m:6.3f} m "
                    f"{format_pose(pose.x_m, pose.y_m, pose.heading_rad)} "
                    f"err={heading_error_deg:+6.2f} deg cmd L={left_command:+5.1f}% R={right_command:+5.1f}%"
                )
                next_print_s = loop_start_s + print_period_s

            sleep(max(0.0, period_s - (monotonic() - loop_start_s)))

        motors.stop()
        final_pose = odom.pose
        print(
            "drive_forward_test complete: "
            f"distance={distance_from_origin(final_pose.x_m, final_pose.y_m):.3f} m "
            f"lateral_y={final_pose.y_m:+.3f} m "
            f"heading={math.degrees(final_pose.heading_rad):+.2f} deg"
        )
        if args.mode == "open-loop":
            print("Open-loop test measured drift only. If it curves, try --mode heading-pid next.")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motors.")
    finally:
        if motors is not None:
            motors.stop()
        if reader is not None:
            reader.close()
        write_csv(args.csv, rows)


if __name__ == "__main__":
    main()
