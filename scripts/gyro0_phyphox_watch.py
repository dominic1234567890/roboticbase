#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
from time import monotonic, sleep

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from tcr_minibot.sensors.phyphox_phone import PhyphoxConfig, PhyphoxError, PhyphoxPhoneGyro


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Watch a phyphox phone gyro yaw-rate buffer from the Raspberry Pi."
    )
    ap.add_argument("--phone-gyro-url", required=True, help="URL shown by phyphox Remote Access, e.g. http://192.168.1.42")
    ap.add_argument("--buffer", default="z", help="phyphox gyro buffer to read; usually z for a flat-mounted phone")
    ap.add_argument("--timeout-s", type=float, default=0.35)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 means run until Ctrl-C")
    ap.add_argument("--calibrate-s", type=float, default=2.0, help="Keep robot still while averaging gyro bias")
    ap.add_argument("--start", action="store_true", help="Send /control?cmd=start before reading")
    ap.add_argument("--clear", action="store_true", help="Send /control?cmd=clear before reading")
    ap.add_argument("--invert", action="store_true", help="Flip yaw-rate sign")
    ap.add_argument("--scale", type=float, default=1.0, help="Multiply yaw-rate by this value after bias removal")
    args = ap.parse_args()

    phone = PhyphoxPhoneGyro(
        PhyphoxConfig(
            base_url=args.phone_gyro_url,
            yaw_rate_buffer=args.buffer,
            timeout_s=args.timeout_s,
            yaw_rate_scale=args.scale,
            invert_yaw_rate=args.invert,
        )
    )

    print(f"Connecting to {phone.base_url}")
    try:
        cfg = phone.get_config()
        title = cfg.get("localTitle") or cfg.get("title") or "unknown experiment"
        names = phone.buffer_names()
        print(f"phyphox experiment: {title}")
        print(f"available buffers: {', '.join(names) if names else '(unknown)'}")
    except PhyphoxError as exc:
        print(f"WARNING: could not fetch phyphox config: {exc}")

    if args.clear:
        print("Clearing phyphox buffers...")
        phone.clear()
    if args.start:
        print("Starting phyphox measurement...")
        phone.start()

    if args.calibrate_s > 0.0:
        print(f"Hold the robot/phone still: calibrating {args.buffer!r} bias for {args.calibrate_s:.1f}s...")
        bias = phone.calibrate_bias(duration_s=args.calibrate_s, sample_hz=max(5.0, args.hz))
        print(f"bias={bias:+.6f} rad/s")

    period_s = 1.0 / max(1.0, args.hz)
    end_s = None if args.seconds <= 0.0 else monotonic() + args.seconds

    print("Turn the robot/phone left and right. Ctrl-C to stop.")
    try:
        while end_s is None or monotonic() < end_s:
            try:
                yaw = phone.read_yaw_rate_radps()
                print(f"yaw_rate={yaw:+.5f} rad/s  {math.degrees(yaw):+7.2f} deg/s")
            except PhyphoxError as exc:
                print(f"phone gyro read failed: {exc}")
            sleep(period_s)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
