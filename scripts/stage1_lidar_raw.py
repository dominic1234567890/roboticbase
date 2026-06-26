#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
try:
    import serial
except Exception as e:
    raise SystemExit("pyserial missing. Install with: pip install pyserial") from e

from tcr_minibot.utils.config import load_config


def main() -> None:
    cfg = load_config()
    ap = argparse.ArgumentParser(description="Stage 1: read raw LD20 bytes")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--chunks", type=int, default=20)
    ap.add_argument("--size", type=int, default=64)
    args = ap.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")
    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        total_headers = 0
        for i in range(args.chunks):
            data = ser.read(args.size)
            total_headers += data.count(bytes([0x54, 0x2C]))
            print(f"{i:02d}: {data.hex(' ')}")
        print(f"\nSaw {total_headers} occurrences of packet header 54 2c in {args.chunks} chunks.")
        print("Good sign: changing hex bytes and repeated 54 2c.")


if __name__ == "__main__":
    main()
