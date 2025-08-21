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
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <gz/utils/ImplPtr.hh>

#include "gz/math/OccupancyGrid.hh"

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
// Private implementation struct/class
struct OccupancyGrid::Implementation {
  double resolutionMeters;
  int widthCells;
  int heightCells;

  // Origin of the grid in world coordinates (bottom-left corner of the grid)
  double originX;
  double originY;

  std::vector<OccupancyCellState> gridData;

  // Helper to get the 1D index from 2D grid coordinates
  int GetIndex(int _gridX, int _gridY) const {
    return _gridY * this->widthCells + _gridX;
  }

  // Constructor for Impl
  Implementation(double _resolutionMeters, int _widthCells, int _heightCells,
    double _originX, double _originY)
    : resolutionMeters(_resolutionMeters),
      widthCells(_widthCells),
      heightCells(_heightCells),
      originX(_originX),
      originY(_originY),
      gridData(_widthCells * _heightCells, OccupancyCellState::Unknown)
  {}
};

/////////////////////////////////////////////////
OccupancyGrid::OccupancyGrid(double resolutionMeters, int widthCells,
  int heightCells, double originX, double originY)
    : pImpl(gz::utils::MakeImpl<Implementation>(
      resolutionMeters, widthCells, heightCells, originX, originY))
{
}

/////////////////////////////////////////////////
bool OccupancyGrid::WorldToGrid(double worldX, double worldY,
  int& gridX, int& gridY) const
{
  if (this->pImpl->resolutionMeters < 1e-6)
  {
    return false;
  }
  gridX = static_cast<int>(
    std::round((worldX - this->pImpl->originX) / this->pImpl->resolutionMeters)
  );
  gridY = static_cast<int>(
    std::round((worldY - this->pImpl->originY) / this->pImpl->resolutionMeters)
  );
  return this->IsValidGridCoordinate(gridX, gridY);
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
  return gridX >= 0 && gridX < this->pImpl->widthCells &&
      gridY >= 0 && gridY < this->pImpl->heightCells;
}

/////////////////////////////////////////////////
OccupancyCellState OccupancyGrid::CellState(int gridX, int gridY) const
{
  if (this->IsValidGridCoordinate(gridX, gridY))
  {
    return this->pImpl->gridData[this->pImpl->GetIndex(gridX, gridY)];
  }
  return OccupancyCellState::Unknown;
}

/////////////////////////////////////////////////
void OccupancyGrid::CellState(int gridX, int gridY, OccupancyCellState state)
{
  if (this->IsValidGridCoordinate(gridX, gridY))
  {
    this->pImpl->gridData[this->pImpl->GetIndex(gridX, gridY)] = state;
  }
}

/////////////////////////////////////////////////
void OccupancyGrid::MarkLine(int x0, int y0, int x1, int y1,
  OccupancyCellState state)
{
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
      auto res = this->CellState(y, x);
      if (res == OccupancyCellState::Occupied)
      {
        continue;
      }
      this->CellState(y, x, state);
    } else {
      auto res = this->CellState(x, y);
      if (res == OccupancyCellState::Occupied)
      {
        continue;
      }
      this->CellState(x, y, state);
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
      auto res = this->CellState(y, x);
      if (res == OccupancyCellState::Occupied ||
        !this->IsValidGridCoordinate(y, x)) {
        return iGain;
      }
      if (res == OccupancyCellState::Unknown) {
        iGain+=1;
      }
    } else {
      auto res = this->CellState(x, y);
      if (res == OccupancyCellState::Occupied ||
        !this->IsValidGridCoordinate(x, y)) {
        return iGain;
      }
      if (res == OccupancyCellState::Unknown) {
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
bool OccupancyGrid::MarkOccupied(double worldX, double worldY)
{
  int gridX, gridY;
  if (WorldToGrid(worldX, worldY, gridX, gridY)) {
    this->CellState(gridX, gridY, OccupancyCellState::Occupied);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool OccupancyGrid::MarkFree(double worldX0, double worldY0, double worldX1,
  double worldY1)
{
  int gridX0, gridY0, gridX1, gridY1;
  if (WorldToGrid(worldX0, worldY0, gridX0, gridY0))
  {
    WorldToGrid(worldX1, worldY1, gridX1, gridY1);
    MarkLine(gridX0, gridY0, gridX1, gridY1, OccupancyCellState::Free);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void OccupancyGrid::ExportToRGBImage(std::vector<uint8_t>& _pixels) const
{
  _pixels.assign(this->pImpl->widthCells * this->pImpl->heightCells * 3, 0);

  for (int gridY = 0; gridY < this->pImpl->heightCells; ++gridY)
  {
    for (int gridX = 0; gridX < this->pImpl->widthCells; ++gridX)
    {
      uint8_t r = 0, g = 0, b = 0;
      auto res = this->CellState(gridX, gridY);
      switch (res)
      {
        case OccupancyCellState::Occupied:
          r = 0; g = 0; b = 0;
          break;
        case OccupancyCellState::Free:
          r = 255; g = 255; b = 255;
          break;
        case OccupancyCellState::Unknown:
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
void OccupancyGrid::RawOccupancy(std::vector<int8_t>& _data) const
{
  _data.assign(this->pImpl->widthCells * this->pImpl->heightCells, 0);

  for (int gridY = 0; gridY < this->pImpl->heightCells; ++gridY)
  {
    for (int gridX = 0; gridX < this->pImpl->widthCells; ++gridX)
    {
      int8_t val = -1;
      auto res = this->CellState(gridX, gridY);
      switch (res)
      {
        case OccupancyCellState::Occupied:
          val = 100;
          break;
        case OccupancyCellState::Free:
          val = 0;
          break;
        case OccupancyCellState::Unknown:
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
double OccupancyGrid::Resolution() const
{
  return this->pImpl->resolutionMeters;
}

/////////////////////////////////////////////////
int OccupancyGrid::Width() const
{
  return this->pImpl->widthCells;
}

/////////////////////////////////////////////////
int OccupancyGrid::Height() const
{
  return this->pImpl->heightCells;
}

/////////////////////////////////////////////////
double OccupancyGrid::OriginX() const
{
  return this->pImpl->originX;
}

/////////////////////////////////////////////////
double OccupancyGrid::OriginY() const
{
  return this->pImpl->originY;
}