from dataclasses import dataclass

from tcr_minibot.perception.point_cloud_export import (
    rows_from_scans,
    write_cloudcompare_ply,
    write_points_csv,
    write_xyz,
)


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float
    distance_m: float
    bearing_deg: float
    confidence: int
    timestamp_ms: int


def test_rows_from_scans_preserves_lidar_fields():
    rows = rows_from_scans([(1, [Point(1.0, 2.0, 2.2, 63.4, 155, 42)])])

    assert len(rows) == 1
    assert rows[0].scan_index == 1
    assert rows[0].z_m == 0.0
    assert rows[0].range_m == 2.2
    assert rows[0].bearing_deg == 63.4
    assert rows[0].confidence == 155
    assert rows[0].timestamp_ms == 42


def test_write_cloudcompare_ply_includes_scalar_fields(tmp_path):
    rows = rows_from_scans([(2, [Point(1.0, -0.5, 1.12, -26.5, 99, 1234)])])
    path = tmp_path / "scan.ply"

    write_cloudcompare_ply(path, rows)

    text = path.read_text(encoding="utf-8")
    assert "format ascii 1.0" in text
    assert "property float range_m" in text
    assert "property int scan_index" in text
    assert "1.000000 -0.500000 0.000000 1.120000 -26.500000 99 2 1234" in text


def test_write_xyz_is_plain_three_column_file(tmp_path):
    rows = rows_from_scans([(1, [Point(1.0, 2.0, 2.2, 63.4, 155, 42)])])
    path = tmp_path / "scan.xyz"

    write_xyz(path, rows)

    assert path.read_text(encoding="utf-8") == "1.000000 2.000000 0.000000\n"


def test_write_points_csv_has_debug_columns(tmp_path):
    rows = rows_from_scans([(1, [Point(1.0, 2.0, 2.2, 63.4, 155, 42)])])
    path = tmp_path / "scan.csv"

    write_points_csv(path, rows)

    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "scan_index,x_m,y_m,z_m,range_m,bearing_deg,confidence,timestamp_ms"
    assert lines[1] == "1,1.000000,2.000000,0.000000,2.200000,63.400000,155,42"
