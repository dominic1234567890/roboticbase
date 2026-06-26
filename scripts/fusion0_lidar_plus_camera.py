#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tcr_minibot.fusion.lidar_camera_fusion import attach_lidar_ranges_to_detections
from tcr_minibot.perception.lidar_filters import aggregate_one_scan, valid_points
from tcr_minibot.perception.vision_simple import contour_detections, draw_detections
from tcr_minibot.sensors.camera_c920 import CameraConfig, USBCamera
from tcr_minibot.sensors.lidar_ld20 import LidarPoint, SerialLD20
from tcr_minibot.utils.config import load_config

try:
    import cv2
except Exception as e:
    raise SystemExit("OpenCV missing. On Pi try: sudo apt install python3-opencv") from e


class SharedLidarScan:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.points: list[LidarPoint] = []
        self.running = True

    def set_points(self, points: list[LidarPoint]) -> None:
        with self.lock:
            self.points = points

    def get_points(self) -> list[LidarPoint]:
        with self.lock:
            return list(self.points)


def lidar_worker(shared: SharedLidarScan, cfg: dict, port: str, baud: int, check_crc: bool) -> None:
    lidar = SerialLD20(port, baud, mount_yaw_offset_deg=cfg["lidar"].get("mount_yaw_offset_deg", 0.0), check_crc=check_crc)
    try:
        while shared.running:
            pts = aggregate_one_scan(lidar.frames(), min_points=360)
            pts = valid_points(pts, min_m=cfg["lidar"].get("min_distance_m", 0.08), max_m=cfg["lidar"].get("max_distance_m", 8.0))
            shared.set_points(pts)
    except Exception as e:
        print(f"LiDAR worker stopped: {e}")
    finally:
        lidar.close()


def main() -> None:
    cfg = load_config()
    cam_cfg = cfg["camera"]
    ap = argparse.ArgumentParser(description="Funky Stage 0: LiDAR-assisted camera detections")
    ap.add_argument("--port", default=cfg["lidar"]["port"])
    ap.add_argument("--baud", type=int, default=cfg["lidar"]["baud"])
    ap.add_argument("--camera", type=int, default=cam_cfg["index"])
    ap.add_argument("--skip-crc", action="store_true")
    ap.add_argument("--min-area", type=float, default=700.0)
    args = ap.parse_args()

    shared = SharedLidarScan()
    t = threading.Thread(target=lidar_worker, args=(shared, cfg, args.port, args.baud, not args.skip_crc), daemon=True)
    t.start()

    cam = USBCamera(CameraConfig(index=args.camera, width=cam_cfg["width"], height=cam_cfg["height"], fps=cam_cfg["fps"]))
    print("Press q to quit. Green boxes show contours with approximate LiDAR range when available.")
    try:
        while True:
            frame = cam.read()
            detections = contour_detections(frame, min_area=args.min_area)[:8]
            points = shared.get_points()
            fused = attach_lidar_ranges_to_detections(
                detections,
                points,
                image_width_px=frame.shape[1],
                horizontal_fov_deg=cam_cfg.get("horizontal_fov_deg", 70.0),
                camera_yaw_offset_deg=cam_cfg.get("mount_yaw_offset_deg", 0.0),
                lidar_tolerance_deg=4.0,
            )
            out = draw_detections(frame, fused)
            cv2.putText(out, f"lidar pts: {len(points)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.imshow("lidar + camera fusion experiment", out)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.005)
    finally:
        shared.running = False
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
