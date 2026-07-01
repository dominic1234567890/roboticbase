from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Any, Iterable

from tcr_minibot.hardware.motors import MotorConfig
from tcr_minibot.odometry.fused_odometry import FusionConfig, WheelGyroOdometry, WheelOdometryConfig
from tcr_minibot.odometry.gpio_quadrature import DriveEncoderPins, GpioZeroEncoderReader, QuadratureEncoderPins


# Fallbacks only. Normal script defaults should come from config/robot.yaml.
DEFAULT_LEFT_A = 27
DEFAULT_LEFT_B = 17
DEFAULT_RIGHT_A = 4
DEFAULT_RIGHT_B = 22
DEFAULT_LEFT_MOTOR_PORT = "M3"
DEFAULT_RIGHT_MOTOR_PORT = "M2"


def _section(cfg: dict[str, Any] | None, name: str) -> dict[str, Any]:
    if not isinstance(cfg, dict):
        return {}
    value = cfg.get(name, {})
    return value if isinstance(value, dict) else {}


def _bool_value(section: dict[str, Any], key: str, default: bool) -> bool:
    value = section.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def add_encoder_pin_args(ap: argparse.ArgumentParser, cfg: dict[str, Any] | None = None) -> None:
    enc_cfg = _section(cfg, "encoders")
    group = ap.add_argument_group("encoder GPIO pins")
    group.add_argument(
        "--left-a",
        type=int,
        default=int(enc_cfg.get("left_channel_a", DEFAULT_LEFT_A)),
        help="BCM GPIO for left encoder channel A; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--left-b",
        type=int,
        default=int(enc_cfg.get("left_channel_b", DEFAULT_LEFT_B)),
        help="BCM GPIO for left encoder channel B; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--right-a",
        type=int,
        default=int(enc_cfg.get("right_channel_a", DEFAULT_RIGHT_A)),
        help="BCM GPIO for right encoder channel A; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--right-b",
        type=int,
        default=int(enc_cfg.get("right_channel_b", DEFAULT_RIGHT_B)),
        help="BCM GPIO for right encoder channel B; default comes from config/robot.yaml",
    )
    group.add_argument("--pull-up", action=argparse.BooleanOptionalAction, default=True, help="Enable gpiozero pullups")
    group.add_argument(
        "--left-invert",
        action=argparse.BooleanOptionalAction,
        default=_bool_value(enc_cfg, "left_inverted", False),
        help="Flip left encoder sign; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--right-invert",
        action=argparse.BooleanOptionalAction,
        default=_bool_value(enc_cfg, "right_inverted", False),
        help="Flip right encoder sign; default comes from config/robot.yaml",
    )


def encoder_pins_from_args(args: argparse.Namespace) -> DriveEncoderPins:
    return DriveEncoderPins(
        left=QuadratureEncoderPins(
            args.left_a,
            args.left_b,
            name="left",
            pull_up=args.pull_up,
            invert=args.left_invert,
        ),
        right=QuadratureEncoderPins(
            args.right_a,
            args.right_b,
            name="right",
            pull_up=args.pull_up,
            invert=args.right_invert,
        ),
    )


def make_encoder_reader(args: argparse.Namespace) -> GpioZeroEncoderReader:
    return GpioZeroEncoderReader(encoder_pins_from_args(args))


def add_odometry_args(ap: argparse.ArgumentParser, cfg: dict[str, Any]) -> None:
    robot_cfg = _section(cfg, "robot")
    enc_cfg = _section(cfg, "encoders")
    odom_cfg = _section(cfg, "odometry")
    group = ap.add_argument_group("odometry calibration")
    group.add_argument("--wheel-radius-m", type=float, default=float(robot_cfg.get("wheel_radius_m", 0.0325)))
    group.add_argument("--wheel-track-m", type=float, default=float(robot_cfg.get("wheel_track_m", 0.1143)))
    group.add_argument(
        "--ticks-per-rev",
        type=int,
        default=int(enc_cfg.get("ticks_per_wheel_rev", 40)),
        help="Measured quadrature counts per one full wheel revolution; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--gyro-delta-weight",
        type=float,
        default=float(odom_cfg.get("gyro_delta_weight", 0.0)),
        help="Future gyro z-rate fusion weight. Leave at 0.0 until the gyro is calibrated.",
    )
    group.add_argument(
        "--absolute-heading-weight",
        type=float,
        default=float(odom_cfg.get("absolute_heading_weight", 0.0)),
        help="Future absolute heading fusion weight. Leave at 0.0 for wheel-only odometry.",
    )


def make_odometry(args: argparse.Namespace) -> WheelGyroOdometry:
    return WheelGyroOdometry(
        wheel_config=WheelOdometryConfig(
            wheel_radius_m=args.wheel_radius_m,
            wheel_track_m=args.wheel_track_m,
            ticks_per_wheel_rev=args.ticks_per_rev,
        ),
        fusion_config=FusionConfig(
            gyro_delta_weight=args.gyro_delta_weight,
            absolute_heading_weight=args.absolute_heading_weight,
        ),
    )


def add_current_motor_args(ap: argparse.ArgumentParser, cfg: dict[str, Any] | None = None) -> None:
    motor_cfg = _section(cfg, "fusion_hat")
    group = ap.add_argument_group("Fusion HAT+ motor ports")
    group.add_argument(
        "--left-motor-port",
        default=str(motor_cfg.get("motor_left_port", DEFAULT_LEFT_MOTOR_PORT)),
        help="Fusion HAT+ port for the physical left motor; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--right-motor-port",
        default=str(motor_cfg.get("motor_right_port", DEFAULT_RIGHT_MOTOR_PORT)),
        help="Fusion HAT+ port for the physical right motor; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--left-motor-reversed",
        action=argparse.BooleanOptionalAction,
        default=_bool_value(motor_cfg, "left_reversed", False),
        help="Reverse the physical left motor direction; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--right-motor-reversed",
        action=argparse.BooleanOptionalAction,
        default=_bool_value(motor_cfg, "right_reversed", False),
        help="Reverse the physical right motor direction; default comes from config/robot.yaml",
    )
    group.add_argument(
        "--max-power",
        type=float,
        default=float(motor_cfg.get("max_test_power_percent", 18.0)),
        help="Hard cap for motor power percent; default comes from config/robot.yaml",
    )


def motor_config_from_args(args: argparse.Namespace) -> MotorConfig:
    return MotorConfig(
        left_port=args.left_motor_port,
        right_port=args.right_motor_port,
        left_reversed=args.left_motor_reversed,
        right_reversed=args.right_motor_reversed,
        max_power_percent=args.max_power,
    )


def describe_motor_args(args: argparse.Namespace) -> str:
    return (
        f"left={args.left_motor_port} reversed={args.left_motor_reversed}  "
        f"right={args.right_motor_port} reversed={args.right_motor_reversed}  "
        f"max_power={args.max_power:.1f}%"
    )


def format_pose(x_m: float, y_m: float, heading_rad: float) -> str:
    return f"x={x_m:+.3f} m y={y_m:+.3f} m heading={math.degrees(heading_rad):+.1f} deg"


@dataclass(frozen=True)
class RouteStep:
    kind: str
    value: float


def parse_route(route_text: str) -> list[RouteStep]:
    """
    Parse commands such as:
      straight:1.0,right:90,straight:1.0,left:90,straight:1.0
    """

    steps: list[RouteStep] = []
    for raw_token in route_text.split(","):
        token = raw_token.strip().lower()
        if not token:
            continue
        if ":" not in token:
            raise ValueError(f"Route token needs name:value form, got {raw_token!r}")
        name, value_text = token.split(":", 1)
        value = float(value_text)
        if name in ("straight", "forward", "f"):
            steps.append(RouteStep("straight", value))
        elif name in ("back", "backward", "reverse"):
            steps.append(RouteStep("straight", -abs(value)))
        elif name in ("left", "l"):
            steps.append(RouteStep("turn", abs(value)))
        elif name in ("right", "r"):
            steps.append(RouteStep("turn", -abs(value)))
        elif name in ("turn", "t"):
            steps.append(RouteStep("turn", value))
        else:
            raise ValueError(f"Unknown route step {name!r}")
    if not steps:
        raise ValueError("Route was empty")
    return steps


def route_summary(steps: Iterable[RouteStep]) -> str:
    parts = []
    for step in steps:
        if step.kind == "straight":
            parts.append(f"straight {step.value:+.3f} m")
        elif step.kind == "turn":
            parts.append(f"turn {step.value:+.1f} deg")
        else:
            parts.append(f"{step.kind}:{step.value}")
    return " -> ".join(parts)
