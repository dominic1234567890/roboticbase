from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from typing import Iterable

from tcr_minibot.odometry.fused_odometry import FusionConfig, WheelGyroOdometry, WheelOdometryConfig
from tcr_minibot.odometry.gpio_quadrature import DriveEncoderPins, GpioZeroEncoderReader, QuadratureEncoderPins


DEFAULT_LEFT_A = 27
DEFAULT_LEFT_B = 17
DEFAULT_RIGHT_A = 4
DEFAULT_RIGHT_B = 22
DEFAULT_LEFT_MOTOR_PORT = "M3"
DEFAULT_RIGHT_MOTOR_PORT = "M2"


def add_encoder_pin_args(ap: argparse.ArgumentParser) -> None:
    group = ap.add_argument_group("encoder GPIO pins")
    group.add_argument("--left-a", type=int, default=DEFAULT_LEFT_A, help="BCM GPIO for left encoder channel A")
    group.add_argument("--left-b", type=int, default=DEFAULT_LEFT_B, help="BCM GPIO for left encoder channel B")
    group.add_argument("--right-a", type=int, default=DEFAULT_RIGHT_A, help="BCM GPIO for right encoder channel A")
    group.add_argument("--right-b", type=int, default=DEFAULT_RIGHT_B, help="BCM GPIO for right encoder channel B")
    group.add_argument("--pull-up", action=argparse.BooleanOptionalAction, default=True, help="Enable gpiozero pullups")
    group.add_argument("--left-invert", action="store_true", help="Flip left encoder sign")
    group.add_argument("--right-invert", action="store_true", help="Flip right encoder sign")


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


def add_odometry_args(ap: argparse.ArgumentParser, cfg: dict) -> None:
    robot_cfg = cfg.get("robot", {})
    enc_cfg = cfg.get("encoders", {})
    group = ap.add_argument_group("odometry calibration")
    group.add_argument("--wheel-radius-m", type=float, default=float(robot_cfg.get("wheel_radius_m", 0.0325)))
    group.add_argument("--wheel-track-m", type=float, default=float(robot_cfg.get("wheel_track_m", 0.1143)))
    group.add_argument(
        "--ticks-per-rev",
        type=int,
        default=int(enc_cfg.get("ticks_per_wheel_rev", 40)),
        help="Measured quadrature counts per one full wheel revolution. Default is only a placeholder.",
    )
    group.add_argument(
        "--gyro-delta-weight",
        type=float,
        default=0.0,
        help="Future gyro z-rate fusion weight. Leave at 0.0 until the gyro is calibrated.",
    )
    group.add_argument(
        "--absolute-heading-weight",
        type=float,
        default=0.0,
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


def add_current_motor_args(ap: argparse.ArgumentParser) -> None:
    group = ap.add_argument_group("Fusion HAT+ motor ports")
    group.add_argument("--left-motor-port", default=DEFAULT_LEFT_MOTOR_PORT)
    group.add_argument("--right-motor-port", default=DEFAULT_RIGHT_MOTOR_PORT)
    group.add_argument("--left-motor-reversed", action="store_true")
    group.add_argument("--right-motor-reversed", action="store_true")
    group.add_argument("--max-power", type=float, default=18.0, help="Hard cap for motor power percent")


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
