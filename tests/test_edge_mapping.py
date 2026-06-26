import math

from tcr_minibot.perception.edge_mapping import EdgeMappingConfig, RoomMapper, XYPoint, extract_edge_segments
from tcr_minibot.perception.occupancy_grid import OccupancyGrid


def _angle_delta_mod_180(a_deg: float, b_deg: float) -> float:
    return abs(((a_deg - b_deg + 90.0) % 180.0) - 90.0)


def _wall_points(
    start: tuple[float, float],
    end: tuple[float, float],
    count: int,
    first_bearing_deg: float,
) -> list[XYPoint]:
    points: list[XYPoint] = []
    for i in range(count):
        t = i / (count - 1)
        x_m = start[0] + (end[0] - start[0]) * t
        y_m = start[1] + (end[1] - start[1]) * t
        points.append(XYPoint(x_m=x_m, y_m=y_m, bearing_deg=first_bearing_deg + i))
    return points


def test_extract_edge_segments_finds_vertical_and_horizontal_walls():
    points = []
    points.extend(_wall_points((2.0, -1.0), (2.0, 1.0), 45, 0.0))
    points.extend(_wall_points((-1.0, 1.6), (1.0, 1.6), 45, 100.0))
    config = EdgeMappingConfig(
        max_neighbor_gap_m=0.2,
        max_line_error_m=0.01,
        min_points_per_segment=8,
        min_segment_length_m=0.5,
    )

    segments = extract_edge_segments(points, config)

    assert len(segments) >= 2
    assert any(segment.length_m > 1.8 and _angle_delta_mod_180(segment.heading_deg, 90.0) < 2.0 for segment in segments)
    assert any(segment.length_m > 1.8 and _angle_delta_mod_180(segment.heading_deg, 0.0) < 2.0 for segment in segments)


def test_room_mapper_add_scan_marks_free_and_occupied_cells():
    grid = OccupancyGrid(size_m=2.0, cell_size_m=0.1)
    mapper = RoomMapper(grid=grid)

    scan = mapper.add_scan([XYPoint(x_m=0.8, y_m=0.0, bearing_deg=0.0)])

    free_cell = grid.world_to_cell(0.4, 0.0)
    occupied_cell = grid.world_to_cell(0.8, 0.0)
    assert free_cell is not None
    assert occupied_cell is not None
    assert grid.grid[free_cell[1], free_cell[0]] < 0
    assert grid.grid[occupied_cell[1], occupied_cell[0]] > 0
    assert len(scan.points) == 1
    assert len(mapper.point_cloud) == 1


def test_room_mapper_rotates_points_with_pose():
    from tcr_minibot.odometry.differential_odometry import Pose2D

    mapper = RoomMapper(grid=OccupancyGrid(size_m=4.0, cell_size_m=0.1))
    scan = mapper.add_scan([XYPoint(x_m=1.0, y_m=0.0, bearing_deg=0.0)], pose=Pose2D(1.0, 2.0, math.pi / 2.0))

    assert abs(scan.points[0].x_m - 1.0) < 1e-6
    assert abs(scan.points[0].y_m - 3.0) < 1e-6
