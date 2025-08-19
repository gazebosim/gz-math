#!/usr/bin/env python3

# Copyright (C) 2025 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from gz.math import OccupancyGrid, CellState

class TestOccupancyGrid(unittest.TestCase):

    def test_constructor(self):
        resolution = 0.1
        width = 10
        height = 20
        originX = -5.0
        originY = -10.0

        grid = OccupancyGrid(resolution, width, height, originX, originY)

        self.assertAlmostEqual(grid.get_resolution(), resolution)
        self.assertEqual(grid.get_width(), width)
        self.assertEqual(grid.get_height(), height)
        self.assertAlmostEqual(grid.get_origin_x(), originX)
        self.assertAlmostEqual(grid.get_origin_y(), originY)

        for y in range(height):
            for x in range(width):
                self.assertEqual(grid.get_cell_state(x, y), CellState.Unknown)

    def test_coordinate_conversions(self):
        grid = OccupancyGrid(0.5, 10, 10, 1.0, 2.0)

        # Test WorldToGrid
        res, grid_x, grid_y = grid.world_to_grid(1.0, 2.0)
        self.assertTrue(res)
        self.assertEqual(grid_x, 0)
        self.assertEqual(grid_y, 0)

        res, grid_x, grid_y = grid.world_to_grid(5.4, 6.4)
        self.assertTrue(res)
        self.assertEqual(grid_x, 9)
        self.assertEqual(grid_y, 9)

        # Test out of bounds
        res, _, _ = grid.world_to_grid(0.0, 0.0)
        self.assertFalse(res)

        # Test GridToWorld
        world_x, world_y = grid.grid_to_world(0, 0)
        self.assertAlmostEqual(world_x, 1.25)
        self.assertAlmostEqual(world_y, 2.25)

        world_x, world_y = grid.grid_to_world(9, 9)
        self.assertAlmostEqual(world_x, 5.75)
        self.assertAlmostEqual(world_y, 6.75)

    def test_cell_state(self):
        grid = OccupancyGrid(1.0, 5, 5)
        self.assertEqual(grid.get_cell_state(2, 2), CellState.Unknown)
        grid.set_cell_state(2, 2, CellState.Occupied)
        self.assertEqual(grid.get_cell_state(2, 2), CellState.Occupied)
        grid.set_cell_state(2, 2, CellState.Free)
        self.assertEqual(grid.get_cell_state(2, 2), CellState.Free)

    def test_mark_occupied_and_free(self):
        grid = OccupancyGrid(0.1, 10, 10)
        grid.mark_occupied(0.55, 0.55)
        res, grid_x, grid_y = grid.world_to_grid(0.55, 0.55)
        self.assertEqual(grid.get_cell_state(grid_x, grid_y), CellState.Occupied)

        grid.mark_free(0.1, 0.1, 0.8, 0.1)
        for i in range(1, 9):
            self.assertEqual(grid.get_cell_state(i, 1), CellState.Free)

    def test_bresenham(self):
        grid = OccupancyGrid(1.0, 10, 10)
        grid.mark_line(0, 0, 5, 5, CellState.Occupied)
        for i in range(6):
            self.assertEqual(grid.get_cell_state(i, i), CellState.Occupied)

    def test_i_gain(self):
        grid = OccupancyGrid(1.0, 10, 10)
        self.assertEqual(grid.calculate_i_gain(0, 0, 5, 0), 6)
        grid.set_cell_state(3, 0, CellState.Occupied)
        self.assertEqual(grid.calculate_i_gain(0, 0, 5, 0), 4)
        grid.set_cell_state(1, 0, CellState.Free)
        self.assertEqual(grid.calculate_i_gain(0, 0, 5, 0), 4)

    def test_export_to_rgb_image(self):
        grid = OccupancyGrid(1.0, 2, 2)
        grid.set_cell_state(0, 0, CellState.Free)
        grid.set_cell_state(0, 1, CellState.Occupied)
        grid.set_cell_state(1, 0, CellState.Unknown)

        pixels = grid.export_to_rgb_image()
        self.assertEqual(len(pixels), 12)

        # (0, 0) - Free (White)
        self.assertEqual(pixels[0:3], b'\xff\xff\xff')
        # (1, 0) - Unknown (Gray)
        self.assertEqual(pixels[3:6], b'\x80\x80\x80')
        # (0, 1) - Occupied (Black)
        self.assertEqual(pixels[6:9], b'\x00\x00\x00')
        # (1, 1) - Unknown (Gray)
        self.assertEqual(pixels[9:12], b'\x80\x80\x80')

    def test_get_raw_occupancy(self):
        grid = OccupancyGrid(1.0, 3, 2)
        grid.set_cell_state(0, 0, CellState.Free)
        grid.set_cell_state(1, 0, CellState.Occupied)
        grid.set_cell_state(2, 0, CellState.Unknown)
        grid.set_cell_state(0, 1, CellState.Occupied)
        grid.set_cell_state(1, 1, CellState.Free)
        grid.set_cell_state(2, 1, CellState.Unknown)

        data = grid.get_raw_occupancy()
        self.assertEqual(len(data), 6)
        expected = bytearray([0, 100, 255, 100, 0, 255])
        for i in range(6):
            self.assertEqual(data[i], expected[i])


if __name__ == '__main__':
    unittest.main()
