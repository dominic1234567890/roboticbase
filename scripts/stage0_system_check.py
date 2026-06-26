#!/usr/bin/env python3
from __future__ import annotations

import glob
import importlib.util
import platform
import subprocess
import sys
from pathlib import Path

# local imports
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.utils.config import load_config


def has_module(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def run(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT, timeout=5).strip()
    except Exception as e:
        return f"not available: {e}"


def main() -> None:
    cfg = load_config()
    print("=== TrashCan Mini system check ===")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.platform()}")
    print(f"Configured LiDAR port: {cfg['lidar']['port']}")
    print(f"Configured LiDAR baud: {cfg['lidar']['baud']}")
    print(f"Configured camera index: {cfg['camera']['index']}")
    print()

    print("Serial devices:", glob.glob("/dev/ttyUSB*") or "none found")
    print("Video devices:", glob.glob("/dev/video*") or "none found")
    print()

    for module in ["serial", "yaml", "numpy", "matplotlib", "cv2"]:
        print(f"module {module:10s}: {'OK' if has_module(module) else 'missing'}")
    print(f"module fusion_hat: {'OK' if has_module('fusion_hat') else 'missing/not installed yet'}")
    print()

    print("v4l2 devices:")
    print(run(["v4l2-ctl", "--list-devices"]))


if __name__ == "__main__":
    main()
