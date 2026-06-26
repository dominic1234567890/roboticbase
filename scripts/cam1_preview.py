#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.sensors.camera_c920 import CameraConfig, USBCamera
from tcr_minibot.utils.config import load_config

try:
    import cv2
except Exception as e:
    raise SystemExit("OpenCV missing. On Pi try: sudo apt install python3-opencv") from e


def main() -> None:
    cfg = load_config()["camera"]
    ap = argparse.ArgumentParser(description="Camera preview")
    ap.add_argument("--camera", type=int, default=cfg["index"])
    args = ap.parse_args()

    cam = USBCamera(CameraConfig(index=args.camera, width=cfg["width"], height=cfg["height"], fps=cfg["fps"]))
    print("Press q to quit.")
    try:
        while True:
            frame = cam.read()
            cv2.imshow("C920 preview", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
