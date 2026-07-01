#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from time import monotonic, sleep
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from _odom_common import add_encoder_pin_args, make_encoder_reader
from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Watch left/right quadrature encoder counts live")
    add_encoder_pin_args(ap, cfg)
    ap.add_argument("--print-hz", type=float, default=4.0)
    ap.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 means forever.")
    args = ap.parse_args()

    period_s = 1.0 / max(0.1, args.print_hz)
    reader = make_encoder_reader(args)
    reader.reset()

    print("Encoder watch started.")
    print(f"Using encoder pins: LEFT=GPIO{args.left_a}/GPIO{args.left_b} RIGHT=GPIO{args.right_a}/GPIO{args.right_b}")
    print(f"Using encoder signs: left_invert={args.left_invert} right_invert={args.right_invert}")
    print("Spin each wheel by hand. Change config/robot.yaml for defaults, or override with CLI flags for one run.")

    start_s = monotonic()
    last_s = start_s
    last_left = 0
    last_right = 0

    try:
        while args.duration <= 0.0 or monotonic() - start_s < args.duration:
            sleep(period_s)
            now_s = monotonic()
            left, right = reader.snapshot()
            dt_s = max(1e-9, now_s - last_s)
            left_rate = (left.count - last_left) / dt_s
            right_rate = (right.count - last_right) / dt_s
            print(
                f"left={left.count:+8d} ({left_rate:+8.1f} tick/s state={left.state:02b} bad={left.bad_transition_count:4d})  "
                f"right={right.count:+8d} ({right_rate:+8.1f} tick/s state={right.state:02b} bad={right.bad_transition_count:4d})"
            )
            last_s = now_s
            last_left = left.count
            last_right = right.count
    except KeyboardInterrupt:
        print("\nStopping encoder watch.")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
