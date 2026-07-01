#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import add_current_motor_args, describe_motor_args, motor_config_from_args
from tcr_minibot.hardware.motors import DifferentialMotors
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Guarded tiny motor pulse test")
    ap.add_argument("--i-understand-motors-are-propped-and-battery-is-plugged", action="store_true", required=True)
    ap.add_argument("--power", type=float, default=12.0)
    ap.add_argument("--seconds", type=float, default=0.20)
    add_current_motor_args(ap, cfg)
    args = ap.parse_args()

    print("Creating motor objects and sending tiny pulse. Keep robot propped up.")
    print(f"Using motor config: {describe_motor_args(args)}")
    motors = DifferentialMotors(motor_config_from_args(args), armed=True)
    try:
        motors.tiny_pulse(power_percent=args.power, seconds=args.seconds)
        print("Done. Motors stopped.")
    finally:
        motors.stop()


if __name__ == "__main__":
    main()
