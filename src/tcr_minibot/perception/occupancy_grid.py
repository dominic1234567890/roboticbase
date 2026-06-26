from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from tcr_minibot.sensors.lidar_ld20 import LidarPoint


@dataclass
class OccupancyGrid:
    size_m: float = 6.0
    cell_size_m: float = 0.05

    def __post_init__(self) -> None:
        self.width = int(self.size_m / self.cell_size_m)
        self.height = int(self.size_m / self.cell_size_m)
        self.origin_cell = (self.width // 2, self.height // 2)
        self.grid = np.zeros((self.height, self.width), dtype=np.int16)

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        cx = int(self.origin_cell[0] + x_m / self.cell_size_m)
        cy = int(self.origin_cell[1] - y_m / self.cell_size_m)
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def clear(self) -> None:
        self.grid.fill(0)

    def mark_free_cell(self, cx: int, cy: int, free_decrement: int = 1) -> None:
        self.grid[cy, cx] = max(-100, self.grid[cy, cx] - free_decrement)

    def mark_occupied_cell(self, cx: int, cy: int, occupied_increment: int = 5) -> None:
        self.grid[cy, cx] = min(100, self.grid[cy, cx] + occupied_increment)

    def cells_on_line(self, start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
        """Integer Bresenham line from start cell to end cell, inclusive."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        cells: list[tuple[int, int]] = []
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return cells
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def add_ray(
        self,
        origin_m: tuple[float, float],
        hit_m: tuple[float, float],
        *,
        free_decrement: int = 1,
        occupied_increment: int = 5,
    ) -> None:
        origin_cell = self.world_to_cell(origin_m[0], origin_m[1])
        hit_cell = self.world_to_cell(hit_m[0], hit_m[1])
        if origin_cell is None or hit_cell is None:
            return

        ray_cells = self.cells_on_line(origin_cell, hit_cell)
        for cx, cy in ray_cells[:-1]:
            self.mark_free_cell(cx, cy, free_decrement=free_decrement)
        end_cx, end_cy = ray_cells[-1]
        self.mark_occupied_cell(end_cx, end_cy, occupied_increment=occupied_increment)

    def add_xy_points(
        self,
        points_xy: Iterable[tuple[float, float]],
        *,
        origin_m: tuple[float, float] = (0.0, 0.0),
        mark_free: bool = True,
        occupied_increment: int = 5,
        free_decrement: int = 1,
    ) -> None:
        for x_m, y_m in points_xy:
            if mark_free:
                self.add_ray(
                    origin_m,
                    (x_m, y_m),
                    free_decrement=free_decrement,
                    occupied_increment=occupied_increment,
                )
                continue

            cell = self.world_to_cell(x_m, y_m)
            if cell is None:
                continue
            cx, cy = cell
            self.mark_occupied_cell(cx, cy, occupied_increment=occupied_increment)

    def add_lidar_points(
        self,
        points: Iterable[LidarPoint],
        occupied_increment: int = 5,
        *,
        origin_m: tuple[float, float] = (0.0, 0.0),
        mark_free: bool = False,
        free_decrement: int = 1,
    ) -> None:
        points_xy = ((p.x_m, p.y_m) for p in points)
        self.add_xy_points(
            points_xy,
            origin_m=origin_m,
            mark_free=mark_free,
            occupied_increment=occupied_increment,
            free_decrement=free_decrement,
        )

    def image_extent_m(self) -> tuple[float, float, float, float]:
        half = self.size_m / 2.0
        return (-half, half, -half, half)

    def as_display_image(self) -> np.ndarray:
        """
        Return an occupancy image where free space is light, occupied is dark,
        and unknown is mid-gray.
        """
        clipped = np.clip(self.grid, -100, 100)
        image = np.full(clipped.shape, 127, dtype=np.uint8)

        free_mask = clipped < 0
        occupied_mask = clipped > 0
        image[free_mask] = np.clip(170 - clipped[free_mask], 170, 255).astype(np.uint8)
        image[occupied_mask] = np.clip(127 - clipped[occupied_mask], 0, 120).astype(np.uint8)
        return image

    def as_uint8_image(self) -> np.ndarray:
        return np.clip(self.grid, 0, 100).astype(np.uint8) * 2
