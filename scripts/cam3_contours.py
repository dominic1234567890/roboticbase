#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.perception.vision_simple import contour_detections, draw_detections
from tcr_minibot.sensors.camera_c920 import CameraConfig, USBCamera
from tcr_minibot.utils.config import load_config

try:
    import cv2
except Exception as e:
    raise SystemExit("OpenCV missing. On Pi try: sudo apt install python3-opencv") from e


def main() -> None:
    cfg = load_config()["camera"]
    ap = argparse.ArgumentParser(description="Simple contour detection stage")
    ap.add_argument("--camera", type=int, default=cfg["index"])
    ap.add_argument("--min-area", type=float, default=700.0)
    args = ap.parse_args()

    cam = USBCamera(CameraConfig(index=args.camera, width=cfg["width"], height=cfg["height"], fps=cfg["fps"]))
    print("Press q to quit.")
    try:
        while True:
            frame = cam.read()
            detections = contour_detections(frame, min_area=args.min_area)
            out = draw_detections(frame, detections[:8])
            cv2.putText(out, f"detections: {len(detections)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("contours", out)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
