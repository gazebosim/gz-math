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

#include <cstdint>
#include <memory>
#include <vector>

#include <gz/math/Helpers.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::math {
inline namespace GZ_MATH_VERSION_NAMESPACE {

  /// \enum OccupancyCellState
  /// \brief Defines the state of a cell in the occupancy grid.
  enum class OccupancyCellState
  {
    /// \brief The cell is considered free space.
    Free,
    /// \brief The cell is considered occupied.
    Occupied,
    /// \brief The state of the cell is unknown.
    Unknown
  };

  /// \class OccupancyGrid OccupancyGrid.hh gz/math/OccupancyGrid.hh
  /// \brief Class representing an occupancy grid.
  class GZ_MATH_VISIBLE OccupancyGrid {
  /// \brief Constructor.
  /// \param[in] _resolutionMeters Resolution of the grid in meters per cell.
  /// \param[in] _widthCells Width of the grid in cells.
  /// \param[in] _heightCells Height of the grid in cells.
  /// \param[in] _originX X-coordinate of the grid's origin in world
  /// coordinates.
  /// \param[in] _originY Y-coordinate of the grid's origin in world
  /// coordinates.
  public: OccupancyGrid(double _resolutionMeters, int _widthCells,
    int _heightCells, double _originX = 0.0, double _originY = 0.0);

  /// \brief Convert world coordinates (meters) to grid coordinates (cells).
  /// \param[in] _worldX World X coordinate in meters.
  /// \param[in] _worldY World Y coordinate in meters.
  /// \param[out] _gridX Grid X coordinate in cells.
  /// \param[out] _gridY Grid Y coordinate in cells.
  /// \return True if coordinates are within bounds, false otherwise. May also
  // be false if resolution is set to zero.
  public: bool WorldToGrid(double _worldX, double _worldY, int &_gridX,
                           int &_gridY) const;

  /// \brief Convert grid coordinates (cells) to world coordinates (meters -
  /// center of cell).
  /// \param[in] _gridX Grid X coordinate in cells.
  /// \param[in] _gridY Grid Y coordinate in cells.
  /// \param[out] _worldX World X coordinate in meters.
  /// \param[out] _worldY World Y coordinate in meters.
  public: void GridToWorld(int _gridX, int _gridY, double &_worldX,
                           double &_worldY) const;

  /// \brief Check if grid coordinates are within bounds.
  /// \param[in] _gridX Grid X coordinate in cells.
  /// \param[in] _gridY Grid Y coordinate in cells.
  /// \return True if the coordinates are valid, false otherwise.
  public: bool IsValidGridCoordinate(int _gridX, int _gridY) const;

  /// \brief Get the state of a cell.
  /// \param[in] _gridX Grid X coordinate in cells.
  /// \param[in] _gridY Grid Y coordinate in cells.
  /// \return The state of the cell.
  public: OccupancyCellState CellState(int _gridX, int _gridY) const;

  /// \brief Set the state of a cell.
  /// \param[in] _gridX Grid X coordinate in cells.
  /// \param[in] _gridY Grid Y coordinate in cells.
  /// \param[in] _state The new state of the cell.
  public: void CellState(int _gridX, int _gridY, OccupancyCellState _state);

  /// \brief Calculate information gain along a line. This is useful when
  /// implementing safe frontier exploration algorithms.
  /// \param[in] _x0 X coordinate of the start point in cells.
  /// \param[in] _y0 Y coordinate of the start point in cells.
  /// \param[in] _x1 X coordinate of the end point in cells.
  /// \param[in] _y1 Y coordinate of the end point in cells.
  /// \return The information gain.
  public: int CalculateIGain(int _x0, int _y0, int _x1, int _y1);

  /// \brief Use Bresenham's Line Algorithm to mark cells along a line. This
  /// function will not modify cells that are already marked as Occupied.
  /// It marks cells from (_x0, _y0) to (_x1, _y1) with the specified state.
  /// \param[in] _x0 X coordinate of the start point in cells.
  /// \param[in] _y0 Y coordinate of the start point in cells.
  /// \param[in] _x1 X coordinate of the end point in cells.
  /// \param[in] _y1 Y coordinate of the end point in cells.
  /// \param[in] _state The state to mark the cells with.
  public: void MarkLine(int _x0, int _y0, int _x1, int _y1,
    OccupancyCellState _state);

  /// \brief Mark a single point as occupied (e.g., an obstacle
  /// detection).
  /// \param[in] _worldX World X coordinate in meters.
  /// \param[in] _worldY World Y coordinate in meters.
  /// \returns true if the grid was inside the coordinates, false otherwise
  public: bool MarkOccupied(double _worldX, double _worldY);

  /// \brief Mark a path as free (e.g., a clear line of sight).
  /// \param[in] _worldX0 World X coordinate of the start point in meters.
  /// \param[in] _worldY0 World Y coordinate of the start point in meters.
  /// \param[in] _worldX1 World X coordinate of the end point in meters.
  /// \param[in] _worldY1 World Y coordinate of the end point in meters.
  /// \returns true if the grid was inside the coordinates, false otherwise
  public: bool MarkFree(double _worldX0, double _worldY0, double _worldX1,
                        double _worldY1);

  /// \brief Export the occupancy grid to a RGB image buffer.
  /// \param[out] _pixels The output buffer to store the RGB image data.
  public: void ExportToRGBImage(std::vector<uint8_t> &_pixels) const;

  /// \brief Export the occupancy grid to a raw buffer.
  /// \param[out] _data The output buffer to store the raw occupancy data.
  public: void RawOccupancy(std::vector<int8_t> &_data) const;

  /// \brief Get the resolution of the occupancy grid.
  /// \return The resolution in meters per cell.
  public: double Resolution() const;

  /// \brief Get the number of cells in width.
  /// \return The width of the grid in cells.
  public: int Width() const;

  /// \brief Get the number of cells in height.
  /// \return The height of the grid in cells.
  public: int Height() const;

  /// \brief Get the origin X position.
  /// \return The X-coordinate of the grid's origin in world coordinates.
  public: double OriginX() const;

  /// \brief Get the origin Y position.
  /// \return The Y-coordinate of the grid's origin in world coordinates.
  public: double OriginY() const;

  /// \brief Private data pointer.
  GZ_UTILS_IMPL_PTR(pImpl)
};
}
}

#endif
