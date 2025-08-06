

/*
 * Copyright (C) 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>
#include "gz/math/OccupancyGrid.hh"

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
TEST(OccupancyGridTest, Constructor)
{
  const double resolution = 0.1;
  const int width = 10;
  const int height = 20;
  const double originX = -5.0;
  const double originY = -10.0;

  OccupancyGrid grid(resolution, width, height, originX, originY);

  EXPECT_DOUBLE_EQ(grid.GetResolution(), resolution);
  EXPECT_EQ(grid.GetWidth(), width);
  EXPECT_EQ(grid.GetHeight(), height);
  EXPECT_DOUBLE_EQ(grid.GetOriginX(), originX);
  EXPECT_DOUBLE_EQ(grid.GetOriginY(), originY);

  // Verify all cells are initialized to Unknown
  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      EXPECT_EQ(grid.GetCellState(x, y), CellState::Unknown);
    }
  }
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, MoveOperations)
{
  OccupancyGrid grid1(0.1, 10, 10);
  grid1.SetCellState(1, 1, CellState::Occupied);

  // Test Move Constructor
  OccupancyGrid grid2(std::move(grid1));
  EXPECT_EQ(grid2.GetCellState(1, 1), CellState::Occupied);

  // grid1 should be in a valid but unspecified state.
  // Let's check if it's still usable.
  // For instance, we could try to re-assign to it.
  grid1 = OccupancyGrid(0.05, 5, 5);
  EXPECT_EQ(grid1.GetWidth(), 5);

  // Test Move Assignment
  OccupancyGrid grid3(0.2, 20, 20);
  grid3.SetCellState(2, 2, CellState::Free);
  grid1 = std::move(grid3);
  EXPECT_EQ(grid1.GetCellState(2, 2), CellState::Free);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, CoordinateConversions)
{
  OccupancyGrid grid(0.5, 10, 10, 1.0, 2.0);

  // Test WorldToGrid
  int gridX, gridY;
  EXPECT_TRUE(grid.WorldToGrid(1.0, 2.0, gridX, gridY));
  EXPECT_EQ(gridX, 0);
  EXPECT_EQ(gridY, 0);

  EXPECT_TRUE(grid.WorldToGrid(5.4, 6.4, gridX, gridY));
  EXPECT_EQ(gridX, 9);
  EXPECT_EQ(gridY, 9);

  // Test out of bounds
  EXPECT_FALSE(grid.WorldToGrid(0.0, 0.0, gridX, gridY));

  // Test GridToWorld
  double worldX, worldY;
  grid.GridToWorld(0, 0, worldX, worldY);
  EXPECT_DOUBLE_EQ(worldX, 1.0);
  EXPECT_DOUBLE_EQ(worldY, 2.0);

  grid.GridToWorld(9, 9, worldX, worldY);
  EXPECT_DOUBLE_EQ(worldX, 5.5);
  EXPECT_DOUBLE_EQ(worldY, 6.5);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, CellState)
{
  OccupancyGrid grid(1.0, 5, 5);

  // Test initial state
  EXPECT_EQ(grid.GetCellState(2, 2), CellState::Unknown);

  // Test setting and getting state
  grid.SetCellState(2, 2, CellState::Occupied);
  EXPECT_EQ(grid.GetCellState(2, 2), CellState::Occupied);

  grid.SetCellState(2, 2, CellState::Free);
  EXPECT_EQ(grid.GetCellState(2, 2), CellState::Free);

  // Test out-of-bounds access
  grid.SetCellState(10, 10, CellState::Occupied); // Should not crash
  EXPECT_EQ(grid.GetCellState(10, 10), CellState::Unknown); // Should return Unknown
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, MarkOccupiedAndFree)
{
  OccupancyGrid grid(0.1, 10, 10);

  // Mark a point as occupied
  grid.MarkOccupied(0.55, 0.55);
  int gridX, gridY;
  grid.WorldToGrid(0.55, 0.55, gridX, gridY);
  EXPECT_EQ(grid.GetCellState(gridX, gridY), CellState::Occupied);

  // Mark a line as free
  grid.MarkFree(0.1, 0.1, 0.8, 0.1);
  for (int i = 1; i <= 8; ++i) {
    EXPECT_EQ(grid.GetCellState(i, 1), CellState::Free);
  }
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, Bresenham)
{
  OccupancyGrid grid(1.0, 10, 10);

  // Mark a diagonal line
  grid.MarkLine(0, 0, 5, 5, CellState::Occupied);
  for (int i = 0; i <= 5; ++i) {
    EXPECT_EQ(grid.GetCellState(i, i), CellState::Occupied);
  }

  // Mark a non-steep line that does not overlap
  grid.MarkLine(0, 6, 8, 8, CellState::Free);
  EXPECT_EQ(grid.GetCellState(0, 6), CellState::Free);
  EXPECT_EQ(grid.GetCellState(1, 6), CellState::Free);
  EXPECT_EQ(grid.GetCellState(2, 6), CellState::Free);
  EXPECT_EQ(grid.GetCellState(3, 7), CellState::Free);
  EXPECT_EQ(grid.GetCellState(4, 7), CellState::Free);
  EXPECT_EQ(grid.GetCellState(5, 7), CellState::Free);
  EXPECT_EQ(grid.GetCellState(6, 7), CellState::Free);
  EXPECT_EQ(grid.GetCellState(7, 8), CellState::Free);
  EXPECT_EQ(grid.GetCellState(8, 8), CellState::Free);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, IGain)
{
  OccupancyGrid grid(1.0, 10, 10);

  // Test on a completely unknown grid
  EXPECT_EQ(grid.CalculateIGain(0, 0, 5, 0), 6);

  // Introduce an obstacle
  grid.SetCellState(3, 0, CellState::Occupied);
  EXPECT_EQ(grid.CalculateIGain(0, 0, 5, 0), 3);

  // Mark some as free
  grid.SetCellState(1, 0, CellState::Free);
  EXPECT_EQ(grid.CalculateIGain(0, 0, 5, 0), 2);

  // Test out of bounds
  EXPECT_EQ(grid.CalculateIGain(0, 0, 15, 0), 10);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, ExportToRGBImage)
{
  OccupancyGrid grid(1.0, 2, 2);
  grid.SetCellState(0, 0, CellState::Free);
  grid.SetCellState(0, 1, CellState::Occupied);
  grid.SetCellState(1, 0, CellState::Unknown);
  // (1, 1) is implicitly Unknown

  std::vector<unsigned char> pixels;
  grid.ExportToRGBImage(pixels);

  // Expected size: 2x2 grid * 3 channels (RGB)
  ASSERT_EQ(pixels.size(), 12);

  // Check pixel colors (in row-major order)
  // (0, 0) - Free (White)
  EXPECT_EQ(pixels[0], 255); EXPECT_EQ(pixels[1], 255); EXPECT_EQ(pixels[2], 255);
  // (1, 0) - Unknown (Gray)
  EXPECT_EQ(pixels[3], 128); EXPECT_EQ(pixels[4], 128); EXPECT_EQ(pixels[5], 128);
  // (0, 1) - Occupied (Black)
  EXPECT_EQ(pixels[6], 0);   EXPECT_EQ(pixels[7], 0);   EXPECT_EQ(pixels[8], 0);
  // (1, 1) - Unknown (Gray)
  EXPECT_EQ(pixels[9], 128); EXPECT_EQ(pixels[10], 128); EXPECT_EQ(pixels[11], 128);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, GetRawOccupancy)
{
  OccupancyGrid grid(1.0, 3, 2); // 3 width, 2 height
  grid.SetCellState(0, 0, CellState::Free);
  grid.SetCellState(1, 0, CellState::Occupied);
  grid.SetCellState(2, 0, CellState::Unknown);
  grid.SetCellState(0, 1, CellState::Occupied);
  grid.SetCellState(1, 1, CellState::Free);
  grid.SetCellState(2, 1, CellState::Unknown);

  std::vector<char> data;
  grid.GetRawOccupancy(data);

  ASSERT_EQ(data.size(), 6);

  // Check values based on CellState enum
  EXPECT_EQ(data[0], 0);
  EXPECT_EQ(data[1], 100);
  EXPECT_EQ(data[2], -1);
  EXPECT_EQ(data[3], 100);
  EXPECT_EQ(data[4], 0);
  EXPECT_EQ(data[5], -1);
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, BoundaryConditions)
{
  OccupancyGrid grid(0.1, 10, 10, -0.5, -0.5);

  int gridX, gridY;
  // Test corners
  EXPECT_TRUE(grid.WorldToGrid(-0.5, -0.5, gridX, gridY));
  EXPECT_EQ(gridX, 0);
  EXPECT_EQ(gridY, 0);

  EXPECT_TRUE(grid.WorldToGrid(0.4, 0.4, gridX, gridY));
  EXPECT_EQ(gridX, 9);
  EXPECT_EQ(gridY, 9);

  // Test just outside boundaries
  EXPECT_FALSE(grid.WorldToGrid(-0.56, -0.5, gridX, gridY));
  EXPECT_FALSE(grid.WorldToGrid(0.45, 0.4, gridX, gridY));

  // Test IsValidGridCoordinate
  EXPECT_TRUE(grid.IsValidGridCoordinate(0, 0));
  EXPECT_TRUE(grid.IsValidGridCoordinate(9, 9));
  EXPECT_FALSE(grid.IsValidGridCoordinate(-1, 5));
  EXPECT_FALSE(grid.IsValidGridCoordinate(5, -1));
  EXPECT_FALSE(grid.IsValidGridCoordinate(10, 5));
  EXPECT_FALSE(grid.IsValidGridCoordinate(5, 10));
}

/////////////////////////////////////////////////
TEST(OccupancyGridTest, MarkLineOverOccupied)
{
  OccupancyGrid grid(1.0, 10, 10);

  grid.SetCellState(3, 3, CellState::Occupied);
  grid.SetCellState(5, 5, CellState::Occupied);

  // Mark a line that passes through the occupied cells
  grid.MarkLine(1, 1, 7, 7, CellState::Free);

  // The occupied cells should remain occupied
  EXPECT_EQ(grid.GetCellState(3, 3), CellState::Occupied);
  EXPECT_EQ(grid.GetCellState(5, 5), CellState::Occupied);

  // Other cells on the line should be free
  EXPECT_EQ(grid.GetCellState(1, 1), CellState::Free);
  EXPECT_EQ(grid.GetCellState(2, 2), CellState::Free);
  EXPECT_EQ(grid.GetCellState(4, 4), CellState::Free);
  EXPECT_EQ(grid.GetCellState(6, 6), CellState::Free);
  EXPECT_EQ(grid.GetCellState(7, 7), CellState::Free);
}
