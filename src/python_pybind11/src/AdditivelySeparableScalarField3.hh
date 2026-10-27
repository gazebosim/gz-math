/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_MATH_PYTHON__ADDITIVELY_SEPARABLE_SCALAR_FIELD3_HH_
#define GZ_MATH_PYTHON__ADDITIVELY_SEPARABLE_SCALAR_FIELD3_HH_

#include <sstream>
#include <string>
#include <tuple>

#include <pybind11/pybind11.h>

#include <gz/math/AdditivelySeparableScalarField3.hh>
#include <gz/math/Polynomial3.hh>
#include <gz/math/Region3.hh>
#include <gz/math/Vector3.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace gz
{
namespace math
{
namespace python
{
/// Help define a pybind11 wrapper for an
/// gz::math::AdditivelySeparableScalarField3 backed by Polynomial3.
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathAdditivelySeparableScalarField3(
    py::module &m, const std::string &typestr)
{
  using Class = gz::math::AdditivelySeparableScalarField3<
      gz::math::Polynomial3<T>, T>;
  auto toString = [](const Class &field)
  {
    std::stringstream stream;
    stream << field;
    return stream.str();
  };

  py::class_<Class>(m, typestr.c_str(), py::dynamic_attr())
    .def(py::init<T, Polynomial3<T>, Polynomial3<T>, Polynomial3<T>>())
    .def("evaluate", &Class::Evaluate,
         "Evaluate the scalar field at `_point`")
    .def("minimum",
         [](const Class &self, const Region3<T> &_region, Vector3<T> &_pMin)
         {
           T result = self.Minimum(_region, _pMin);
           return std::make_tuple(result, _pMin);
         },
         "Compute the scalar field minimum in a region")
    .def("minimum",
         py::overload_cast<const Region3<T> &>(
             &Class::Minimum, py::const_),
         "Compute the scalar field minimum in a region")
    .def("minimum",
         [](const Class &self, Vector3<T> &_pMin)
         {
           T result = self.Minimum(_pMin);
           return std::make_tuple(result, _pMin);
         },
         "Compute the scalar field minimum")
    .def("minimum",
         py::overload_cast<>(&Class::Minimum, py::const_),
         "Compute the scalar field minimum")
    .def("__call__", &Class::operator())
    .def("__copy__", [](const Class &self)
    {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict)
    {
      return Class(self);
    }, "memo"_a)
    .def("__str__", toString)
    .def("__repr__", toString);
}

/// Define pybind11 wrappers for polynomial-backed
/// gz::math::AdditivelySeparableScalarField3 types.
/**
 * \param[in] module a pybind11 module to add the definitions to
 * \param[in] typestr base name of the types used by Python
 */
void defineMathAdditivelySeparableScalarField3(
    py::module &m, const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__ADDITIVELY_SEPARABLE_SCALAR_FIELD3_HH_
