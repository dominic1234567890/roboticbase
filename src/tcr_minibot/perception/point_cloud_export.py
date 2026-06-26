from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Protocol


class ExportPointLike(Protocol):
    x_m: float
    y_m: float


@dataclass(frozen=True)
class PointCloudRow:
    scan_index: int
    x_m: float
    y_m: float
    z_m: float = 0.0
    range_m: float | None = None
    bearing_deg: float | None = None
    confidence: int | None = None
    timestamp_ms: int | None = None


def rows_from_scans(scans: Iterable[tuple[int, Iterable[ExportPointLike]]]) -> list[PointCloudRow]:
    rows: list[PointCloudRow] = []
    for scan_index, points in scans:
        for point in points:
            rows.append(
                PointCloudRow(
                    scan_index=scan_index,
                    x_m=float(point.x_m),
                    y_m=float(point.y_m),
                    range_m=_float_attr(point, "distance_m"),
                    bearing_deg=_float_attr(point, "bearing_deg"),
                    confidence=_int_attr(point, "confidence"),
                    timestamp_ms=_int_attr(point, "timestamp_ms"),
                )
            )
    return rows


def write_cloudcompare_ply(path: str | Path, rows: Iterable[PointCloudRow]) -> None:
    pts = list(rows)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment LD20 horizontal scan export from trashcan-mini-pi5-robot\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property float range_m\n")
        f.write("property float bearing_deg\n")
        f.write("property int confidence\n")
        f.write("property int scan_index\n")
        f.write("property int timestamp_ms\n")
        f.write("end_header\n")
        for row in pts:
            f.write(
                f"{row.x_m:.6f} {row.y_m:.6f} {row.z_m:.6f} "
                f"{_num(row.range_m):.6f} {_num(row.bearing_deg):.6f} "
                f"{_int(row.confidence)} {row.scan_index} {_int(row.timestamp_ms)}\n"
            )


def write_xyz(path: str | Path, rows: Iterable[PointCloudRow]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        for row in rows:
            f.write(f"{row.x_m:.6f} {row.y_m:.6f} {row.z_m:.6f}\n")


def write_points_csv(path: str | Path, rows: Iterable[PointCloudRow]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["scan_index", "x_m", "y_m", "z_m", "range_m", "bearing_deg", "confidence", "timestamp_ms"])
        for row in rows:
            writer.writerow(
                [
                    row.scan_index,
                    f"{row.x_m:.6f}",
                    f"{row.y_m:.6f}",
                    f"{row.z_m:.6f}",
                    "" if row.range_m is None else f"{row.range_m:.6f}",
                    "" if row.bearing_deg is None else f"{row.bearing_deg:.6f}",
                    "" if row.confidence is None else row.confidence,
                    "" if row.timestamp_ms is None else row.timestamp_ms,
                ]
            )


def _float_attr(point: ExportPointLike, attr: str) -> float | None:
    value = getattr(point, attr, None)
    return float(value) if isinstance(value, (int, float)) else None


def _int_attr(point: ExportPointLike, attr: str) -> int | None:
    value = getattr(point, attr, None)
    return int(value) if isinstance(value, int) else None


def _num(value: float | None) -> float:
    return 0.0 if value is None else value


def _int(value: int | None) -> int:
    return 0 if value is None else value
