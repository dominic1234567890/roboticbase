from __future__ import annotations

from dataclasses import dataclass

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None


@dataclass(frozen=True)
class Detection2D:
    x: int
    y: int
    w: int
    h: int
    area: float
    label: str = "contour"
    range_m: float | None = None
    bearing_deg: float | None = None

    @property
    def cx(self) -> float:
        return self.x + self.w / 2

    @property
    def cy(self) -> float:
        return self.y + self.h / 2


def require_cv2():
    if cv2 is None:
        raise RuntimeError("OpenCV is not installed. On Pi try: sudo apt install python3-opencv")
    return cv2


def edges(frame, low_threshold: int = 80, high_threshold: int = 160):
    cv = require_cv2()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    return cv.Canny(blur, low_threshold, high_threshold)


def contour_detections(frame, min_area: float = 700.0) -> list[Detection2D]:
    cv = require_cv2()
    edge_img = edges(frame)
    contours, _ = cv.findContours(edge_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    detections: list[Detection2D] = []
    for c in contours:
        area = cv.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv.boundingRect(c)
        detections.append(Detection2D(x=x, y=y, w=w, h=h, area=float(area)))
    return sorted(detections, key=lambda d: d.area, reverse=True)


def draw_detections(frame, detections: list[Detection2D]):
    cv = require_cv2()
    out = frame.copy()
    for d in detections:
        cv.rectangle(out, (d.x, d.y), (d.x + d.w, d.y + d.h), (0, 255, 0), 2)
        text = d.label
        if d.range_m is not None:
            text += f" {d.range_m:.2f}m"
        if d.bearing_deg is not None:
            text += f" {d.bearing_deg:+.1f}deg"
        cv.putText(out, text, (d.x, max(20, d.y - 8)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out
