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

#include "gz/math/OccupancyGrid.hh"
#include <cmath>
#include <utility>

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
// Private implementation struct/class
struct OccupancyGrid::Impl {
  double resolutionMeters;
  int widthCells;
  int heightCells;

  // Origin of the grid in world coordinates (bottom-left corner of the grid)
  double originX;
  double originY;

  std::vector<CellState> gridData;

  // Helper to get the 1D index from 2D grid coordinates
  int GetIndex(int _gridX, int _gridY) const {
    return _gridY * this->widthCells + _gridX;
  }

  // Constructor for Impl
  Impl(double _resolutionMeters, int _widthCells, int _heightCells,
    double _originX, double _originY)
    : resolutionMeters(_resolutionMeters),
      widthCells(_widthCells),
      heightCells(_heightCells),
      originX(_originX),
      originY(_originY),
      gridData(_widthCells * _heightCells, CellState::Unknown)
  {}

  // IsValidGridCoordinate is also needed internally for Impl operations
  bool IsValidGridCoordinateImpl(int gridX, int gridY) const {
    return gridX >= 0 && gridX < this->widthCells &&
      gridY >= 0 && gridY < this->heightCells;
  }

  // Get and Set CellState are also needed internally
  CellState GetCellStateImpl(int gridX, int gridY) const
  {
    if (this->IsValidGridCoordinateImpl(gridX, gridY))
    {
      return this->gridData[GetIndex(gridX, gridY)];
    }
    return CellState::Unknown;
  }

  void SetCellStateImpl(int gridX, int gridY, CellState state)
  {
    if (this->IsValidGridCoordinateImpl(gridX, gridY))
    {
      this->gridData[GetIndex(gridX, gridY)] = state;
    }
  }
};

/////////////////////////////////////////////////
OccupancyGrid::OccupancyGrid(double resolutionMeters, int widthCells,
  int heightCells, double originX, double originY)
    : pImpl(std::make_unique<Impl>(
      resolutionMeters, widthCells, heightCells, originX, originY))
{
}

/////////////////////////////////////////////////
OccupancyGrid::~OccupancyGrid() = default;

/////////////////////////////////////////////////
OccupancyGrid::OccupancyGrid(OccupancyGrid&& other) noexcept
    : pImpl(std::move(other.pImpl))
{}

/////////////////////////////////////////////////
OccupancyGrid& OccupancyGrid::operator=(OccupancyGrid&& other) noexcept
{
  if (this != &other)
  {
    pImpl = std::move(other.pImpl);
  }
  return *this;
}

/////////////////////////////////////////////////
bool OccupancyGrid::WorldToGrid(double worldX, double worldY,
  int& gridX, int& gridY) const
{
  gridX = static_cast<int>(
    std::round((worldX - this->pImpl->originX) / this->pImpl->resolutionMeters)
  );
  gridY = static_cast<int>(
    std::round((worldY - this->pImpl->originY) / this->pImpl->resolutionMeters)
  );
  return this->pImpl->IsValidGridCoordinateImpl(gridX, gridY);
}

/////////////////////////////////////////////////
void OccupancyGrid::GridToWorld(int gridX, int gridY,
  double& worldX, double& worldY) const
{
  worldX = gridX * this->pImpl->resolutionMeters + this->pImpl->originX;
  worldY = gridY * this->pImpl->resolutionMeters + this->pImpl->originY;
}

/////////////////////////////////////////////////
bool OccupancyGrid::IsValidGridCoordinate(int gridX, int gridY) const
{
  return this->pImpl->IsValidGridCoordinateImpl(gridX, gridY);
}

/////////////////////////////////////////////////
CellState OccupancyGrid::GetCellState(int gridX, int gridY) const
{
  return this->pImpl->GetCellStateImpl(gridX, gridY);
}

/////////////////////////////////////////////////
void OccupancyGrid::SetCellState(int gridX, int gridY, CellState state)
{
  this->pImpl->SetCellStateImpl(gridX, gridY, state);
}

/////////////////////////////////////////////////
void OccupancyGrid::MarkLine(int x0, int y0, int x1, int y1, CellState state)
{
  // Bresenham logic now operates on the internal Impl's data and methods
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);

  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int error = dx / 2;
  int yStep = (y0 < y1) ? 1 : -1;
  int y = y0;

  for (int x = x0; x <= x1; ++x) {
    if (steep) {
      auto res = this->pImpl->GetCellStateImpl(y, x);
      if (res == CellState::Occupied)
      {
        continue;
      }
      this->pImpl->SetCellStateImpl(y, x, state);
    } else {
      auto res = this->pImpl->GetCellStateImpl(x, y);
      if (res == CellState::Occupied)
      {
        continue;
      }
      this->pImpl->SetCellStateImpl(x, y, state);
    }
    error -= dy;
    if (error < 0) {
      y += yStep;
      error += dx;
    }
  }
}

/////////////////////////////////////////////////
int OccupancyGrid::CalculateIGain(int x0, int y0, int x1, int y1)
{
  // Bresenham logic now operates on the internal Impl's data and methods
  bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);

  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int dx = x1 - x0;
  int dy = std::abs(y1 - y0);
  int error = dx / 2;
  int yStep = (y0 < y1) ? 1 : -1;
  int y = y0;

  int iGain = 0;

  for (int x = x0; x <= x1; ++x) {
    if (steep) {
      auto res = this->pImpl->GetCellStateImpl(y, x);
      if (res == CellState::Occupied ||
        !this->IsValidGridCoordinate(y, x)) {
        return iGain;
      }
      if (res == CellState::Unknown) {
        iGain+=1;
      }
    } else {
      auto res = this->pImpl->GetCellStateImpl(x, y);
      if (res == CellState::Occupied ||
        !this->IsValidGridCoordinate(x, y)) {
        return iGain;
      }
      if (res == CellState::Unknown) {
        iGain+=1;
      }
    }
    error -= dy;
    if (error < 0) {
      y += yStep;
      error += dx;
    }
  }
  return iGain;
}

/////////////////////////////////////////////////
void OccupancyGrid::MarkOccupied(double worldX, double worldY)
{
  int gridX, gridY;
  if (WorldToGrid(worldX, worldY, gridX, gridY)) {
    this->pImpl->SetCellStateImpl(gridX, gridY, CellState::Occupied);
  }
}

/////////////////////////////////////////////////
void OccupancyGrid::MarkFree(double worldX0, double worldY0, double worldX1,
  double worldY1)
{
  int gridX0, gridY0, gridX1, gridY1;
  if (WorldToGrid(worldX0, worldY0, gridX0, gridY0))
  {
    WorldToGrid(worldX1, worldY1, gridX1, gridY1);
    MarkLine(gridX0, gridY0, gridX1, gridY1, CellState::Free);
  }
}

/////////////////////////////////////////////////
void OccupancyGrid::ExportToRGBImage(std::vector<unsigned char>& _pixels) const
{
  _pixels.assign(this->pImpl->widthCells * this->pImpl->heightCells * 3, 0);

  for (int gridY = 0; gridY < this->pImpl->heightCells; ++gridY)
  {
    for (int gridX = 0; gridX < this->pImpl->widthCells; ++gridX)
    {
      unsigned char r = 0, g = 0, b = 0;
      auto res = this->pImpl->GetCellStateImpl(gridX, gridY);
      switch (res)
      {
        case CellState::Occupied:
          r = 0; g = 0; b = 0;
          break;
        case CellState::Free:
          r = 255; g = 255; b = 255;
          break;
        case CellState::Unknown:
          r = 128; g = 128; b = 128;
          break;
        default:
          // Should never reach here
          r = 128; g = 128; b = 128;
          break;
      }

      int pixelIdx = (gridY * this->pImpl->widthCells + gridX) * 3;

      _pixels[pixelIdx + 0] = r;
      _pixels[pixelIdx + 1] = g;
      _pixels[pixelIdx + 2] = b;

    }
  }
}

/////////////////////////////////////////////////
void OccupancyGrid::GetRawOccupancy(std::vector<char>& _data) const
{
  _data.assign(this->pImpl->widthCells * this->pImpl->heightCells, 0);

  for (int gridY = 0; gridY < this->pImpl->heightCells; ++gridY)
  {
    for (int gridX = 0; gridX < this->pImpl->widthCells; ++gridX)
    {
      char val = -1;
      auto res = this->pImpl->GetCellStateImpl(gridX, gridY);
      switch (res)
      {
        case CellState::Occupied:
          val = 100;
          break;
        case CellState::Free:
          val = 0;
          break;
        case CellState::Unknown:
          val = -1;
          break;
        default:
          val = -1;
          break;
      }

      int pixelIdx = (gridY * this->pImpl->widthCells + gridX);
      _data[pixelIdx] = val;
    }
  }
}

/////////////////////////////////////////////////
double OccupancyGrid::GetResolution() const
{
  return this->pImpl->resolutionMeters;
}

/////////////////////////////////////////////////
int OccupancyGrid::GetWidth() const
{
  return this->pImpl->widthCells;
}

/////////////////////////////////////////////////
int OccupancyGrid::GetHeight() const
{
  return this->pImpl->heightCells;
}

/////////////////////////////////////////////////
double OccupancyGrid::GetOriginX() const
{
  return this->pImpl->originX;
}

/////////////////////////////////////////////////
double OccupancyGrid::GetOriginY() const
{
  return this->pImpl->originY;
}
