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

#ifndef GZ_MATH_PYTHON__INTERPOLATION_POINT_HH_
#define GZ_MATH_PYTHON__INTERPOLATION_POINT_HH_

#include <sstream>
#include <string>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/math/Vector3.hh>
#include <gz/math/detail/InterpolationPoint.hh>
#include <optional>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz {
namespace math {
namespace python {
/// define a pybind11 wrapper for gz::math::InterpolationPoint3D
/**
 * \param[in] module a pybind11 module to add the definition to
 */
template <typename T>
void helpDefineMathInterpolationPoint3D(py::module &m,
                                        const std::string &typestr) {
  using Class = gz::math::InterpolationPoint3D<T>;
  using Vector3Type = gz::math::Vector3<T>;

  auto toString = [](const Class &si) {
    std::stringstream stream;
    stream << "InterpolationPoint3D(position=" << si.position;
    if (si.index.has_value()) {
      stream << ", index=" << si.index.value();
    } else {
      stream << ", index=None";
    }
    stream << ")";
    return stream.str();
  };

  std::string pyclass_name = typestr;
  py::class_<Class>(m, pyclass_name.c_str(), py::dynamic_attr())
      .def(py::init<>())
      .def(py::init<Vector3Type, std::optional<std::size_t>>(),
           py::arg("position"), py::arg("index") = std::nullopt)
      .def_readwrite("position", &Class::position)
      .def_readwrite("index", &Class::index)
      .def("__str__", toString)
      .def("__repr__", toString);
}

/// define a pybind11 wrapper for gz::math::InterpolationPoint3D
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineMathInterpolationPoint3D(py::module &m, const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__INTERPOLATION_POINT_HH_
