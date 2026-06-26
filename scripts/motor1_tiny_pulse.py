#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.hardware.motors import DifferentialMotors, MotorConfig
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()["fusion_hat"]
    ap = argparse.ArgumentParser(description="Guarded tiny motor pulse test")
    ap.add_argument("--i-understand-motors-are-propped-and-battery-is-plugged", action="store_true", required=True)
    ap.add_argument("--power", type=float, default=12.0)
    ap.add_argument("--seconds", type=float, default=0.20)
    args = ap.parse_args()

    print("Creating motor objects and sending tiny pulse. Keep robot propped up.")
    motors = DifferentialMotors(
        MotorConfig(
            left_port=cfg.get("motor_left_port", "M0"),
            right_port=cfg.get("motor_right_port", "M1"),
            left_reversed=cfg.get("left_reversed", False),
            right_reversed=cfg.get("right_reversed", True),
            max_power_percent=cfg.get("max_test_power_percent", 20),
        ),
        armed=True,
    )
    try:
        motors.tiny_pulse(power_percent=args.power, seconds=args.seconds)
        print("Done. Motors stopped.")
    finally:
        motors.stop()


if __name__ == "__main__":
    main()
