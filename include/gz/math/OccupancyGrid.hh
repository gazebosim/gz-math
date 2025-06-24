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
#ifndef GZ_MATH_OCCUPANCY_GRID_HH_
#define GZ_MATH_OCCUPANCY_GRID_HH_

#include <memory>
#include <vector>

#include <gz/math/Helpers.hh>

namespace gz::math {
inline namespace GZ_MATH_VERSION_NAMESPACE {
// Define cell states
enum class CellState {
  Free,
  Occupied,
  Unknown
};

/// Class representing an occupancy grid.
class GZ_MATH_VISIBLE OccupancyGrid {
public:
  /// Constructor
  OccupancyGrid(double resolutionMeters, int widthCells,
    int heightCells, double originX = 0.0, double originY = 0.0);

  /// Destructor (important for PIMPL)
  ~OccupancyGrid();

  /// Copy Constructor and Assignment Operator (deleted for to prevent expensive
  /// copies)
  OccupancyGrid(const OccupancyGrid&) = delete;
  OccupancyGrid& operator=(const OccupancyGrid&) = delete;

  /// Move Constructor and Assignment Operator
  OccupancyGrid(OccupancyGrid&&) noexcept;
  OccupancyGrid& operator=(OccupancyGrid&&) noexcept;

  /// Convert world coordinates (meters) to grid coordinates (cells)
  bool WorldToGrid(double worldX, double worldY, int& gridX, int& gridY) const;

  /// Convert grid coordinates (cells) to world coordinates
  /// (meters - center of cell)
  void GridToWorld(int gridX, int gridY, double& worldX, double& worldY) const;

  /// Check if grid coordinates are within bounds
  bool IsValidGridCoordinate(int gridX, int gridY) const;

  /// Get the state of a cell
  CellState GetCellState(int gridX, int gridY) const;

  /// Set the state of a cell
  void SetCellState(int gridX, int gridY, CellState state);

  /// Calculate information gain along a line. Useful when implementing
  /// safe frontier exploration algorithms.
  double CalculateIGain(int x0, int y0, int x1, int y1);

  /// Bresenham's Line Algorithm to mark cells along a line
  /// Marks cells from (x0, y0) to (x1, y1) with the specified state
  void MarkLine(int x0, int y0, int x1, int y1, CellState state);

  /// Helper to mark a single point as occupied (e.g., an obstacle detection)
  void MarkOccupied(double worldX, double worldY);

  /// Helper to mark a path as free (e.g., a clear line of sight)
  void MarkFree(double worldX0, double worldY0, double worldX1, double worldY1);

  /// Export the occupancy grid to a RGB image buffer
  void ExportToRGBImage(std::vector<unsigned char>& _pixels) const;

  /// Export the occupancy grid to a raw buffer
  void GetRawOccupancy(std::vector<char>& _data) const;

  /// Resolution of the occupancy grid
  double GetResolution() const;

  /// Get the number of cells in width
  int GetWidth() const;

  /// Get the number of cells in height
  int GetHeight() const;

  /// Get the origin X position
  double GetOriginX() const;

  /// Get the origin Y position
  double GetOriginY() const;
private:
  // PIMPL idiom: Pointer to implementation
  struct Impl;
  std::unique_ptr<Impl> pImpl;
};
}
}

#endif
