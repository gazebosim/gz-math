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

#ifndef GZ_MATH_PYTHON__VOLUMETRIC_GRID_LOOKUP_FIELD_HH_
#define GZ_MATH_PYTHON__VOLUMETRIC_GRID_LOOKUP_FIELD_HH_

#include <sstream>
#include <string>
#include <vector>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/math/Vector3.hh>
#include <gz/math/VolumetricGridLookupField.hh>
#include <gz/math/detail/InterpolationPoint.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz {
namespace math {
namespace python {
/// define a pybind11 wrapper for a gz::math::VolumetricGridLookupField
/**
 * \param[in] module a pybind11 module to add the definition to
 */
template <typename T, typename I = std::size_t>
void helpDefineMathVolumetricGridLookupField(py::module &m,
                                             const std::string &typestr) {
  using Class = gz::math::VolumetricGridLookupField<T, I>;
  using Vector3Type = gz::math::Vector3<T>;

  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << si;
    return stream.str();
  };

  std::string pyclass_name = typestr;
  py::class_<Class>(m, pyclass_name.c_str(), py::buffer_protocol(),
                    py::dynamic_attr())
      .def(py::init<const std::vector<Vector3Type> &>())
      .def(py::init<const std::vector<Vector3Type> &, const std::vector<I> &>())
      .def(
          "get_interpolators",
          [](const Class &self, const Vector3Type &pt, double xTol, double yTol,
             double zTol) {
            return self.GetInterpolators(pt, xTol, yTol, zTol);
          },
          "Get interpolators for a given point", py::arg("point"),
          py::arg("x_tol") = 1e-6, py::arg("y_tol") = 1e-6,
          py::arg("z_tol") = 1e-6)
      .def(
          "estimate_value_using_trilinear",
          [](const Class &self, const Vector3Type &pt,
             const std::vector<double> &values, double defaultVal = 0.0) {
            return self.EstimateValueUsingTrilinear(pt, values, defaultVal);
          },
          "Estimate value using trilinear interpolation", py::arg("point"),
          py::arg("values"), py::arg("default") = 0.0)
      .def("bounds", &Class::Bounds, "Get the bounds of the grid field")
      .def("__str__", toString)
      .def("__repr__", toString);
}

/// Define a pybind11 wrapper for a gz::math::VolumetricGridLookupField
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineMathVolumetricGridLookupField(py::module &m,
                                         const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__VOLUMETRIC_GRID_LOOKUP_FIELD_HH_
