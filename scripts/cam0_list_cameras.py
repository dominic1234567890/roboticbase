#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.sensors.camera_c920 import list_camera_indices


def main() -> None:
    try:
        print("OpenCV camera indices:", list_camera_indices())
    except Exception as e:
        print(f"OpenCV camera scan failed: {e}")

    print("\nv4l2-ctl output:")
    try:
        print(subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True))
    except Exception as e:
        print(f"v4l2-ctl not available: {e}")


if __name__ == "__main__":
    main()
