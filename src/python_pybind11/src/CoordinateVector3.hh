/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef GZ_MATH_PYTHON__COORDINATE_VECTOR3_HH_
#define GZ_MATH_PYTHON__COORDINATE_VECTOR3_HH_

#include <string>

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{

/// Define a pybind11 wrapper for a gz::math::CoordinateVector3
/**
 * \param[in] module a pybind11 module to add the definition to
 */
void defineMathCoordinateVector3(py::module &m, const std::string &typestr);
}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__COORDINATE_VECTOR3_HH_
