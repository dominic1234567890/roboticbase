from __future__ import annotations

from dataclasses import dataclass

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None


@dataclass
class CameraConfig:
    index: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30


class USBCamera:
    def __init__(self, cfg: CameraConfig) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is not installed. On Pi try: sudo apt install python3-opencv")
        self.cfg = cfg
        self.cap = cv2.VideoCapture(cfg.index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {cfg.index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        self.cap.set(cv2.CAP_PROP_FPS, cfg.fps)

    def read(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("Camera read failed")
        return frame

    def release(self) -> None:
        self.cap.release()


def list_camera_indices(max_index: int = 8) -> list[int]:
    if cv2 is None:
        raise RuntimeError("OpenCV is not installed. On Pi try: sudo apt install python3-opencv")
    found: list[int] = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(idx)
        cap.release()
    return found
