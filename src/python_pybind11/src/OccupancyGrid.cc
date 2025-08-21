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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "OccupancyGrid.hh"

#include <gz/math/OccupancyGrid.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
void defineMathOccupancyGrid(py::module &m, const std::string &typestr)
{
  using Class = gz::math::OccupancyGrid;
  py::class_<Class, std::unique_ptr<Class>>(m, typestr.c_str())
    .def(py::init<double, int, int, double, double>(),
         "resolutionMeters"_a, "widthCells"_a, "heightCells"_a,
         "originX"_a = 0.0, "originY"_a = 0.0)
    .def("world_to_grid",
         [](const Class &self, double worldX, double worldY) {
           int gridX, gridY;
           bool ok = self.WorldToGrid(worldX, worldY, gridX, gridY);
           return std::make_tuple(ok, gridX, gridY);
         },
         "Convert world coordinates to grid coordinates.")
    .def("grid_to_world",
        [](const Class &self, int gridX, int gridY) {
            double worldX, worldY;
            self.GridToWorld(gridX, gridY, worldX, worldY);
            return std::make_tuple(worldX, worldY);
        },
         "Convert grid coordinates to world coordinates.")
    .def("is_valid_grid_coordinate", &Class::IsValidGridCoordinate,
         "Check if grid coordinates are within bounds.")
    .def("get_cell_state", &Class::GetCellState,
         "Get the state of a cell.")
    .def("set_cell_state", &Class::SetCellState,
         "Set the state of a cell.")
    .def("calculate_i_gain", &Class::CalculateIGain,
         "Calculate information gain along a line.")
    .def("mark_line", &Class::MarkLine,
         "Mark cells along a line with the specified state.")
    .def("mark_occupied", &Class::MarkOccupied,
         "Mark a single point as occupied.")
    .def("mark_free", &Class::MarkFree,
         "Mark a path as free.")
    .def("export_to_rgb_image", [](const Class &self) {
        std::vector<uint8_t> pixels;
        self.ExportToRGBImage(pixels);
        return py::bytes(reinterpret_cast<const char*>(pixels.data()),
          pixels.size());
     }, "Export the occupancy grid to a RGB image buffer.")
    .def("get_raw_occupancy", [](const Class &self) {
        std::vector<int8_t> data;
        self.GetRawOccupancy(data);
        return py::bytes(reinterpret_cast<const char*>(data.data()),
          data.size());
     }, "Export the occupancy grid to a raw buffer.")
    .def("get_resolution", &Class::GetResolution,
         "Get the resolution of the occupancy grid.")
    .def("get_width", &Class::GetWidth,
         "Get the number of cells in width.")
    .def("get_height", &Class::GetHeight,
         "Get the number of cells in height.")
    .def("get_origin_x", &Class::GetOriginX,
         "Get the origin X position.")
    .def("get_origin_y", &Class::GetOriginY,
         "Get the origin Y position.");

  py::enum_<gz::math::CellState>(m, "CellState")
    .value("Free", gz::math::CellState::Free)
    .value("Occupied", gz::math::CellState::Occupied)
    .value("Unknown", gz::math::CellState::Unknown)
    .export_values();
}
}  // namespace python
}  // namespace math
}  // namespace gz
