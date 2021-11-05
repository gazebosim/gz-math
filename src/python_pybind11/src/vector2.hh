// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef IGNITION_MATH_PYTHON__VECTOR2D_HPP_
#define IGNITION_MATH_PYTHON__VECTOR2D_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ignition/math/Vector2.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an ignition::gazebo::Vector2
/**
 * \param[in] module a pybind11 module to add the definition to
 */
template<typename T>
void define_math_vector2(py::module &m, const std::string &typestr)
{
  using Class = ignition::math::Vector2<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const T&, const T&>())
    .def(py::init<const Class>())
    .def("sum", &Class::Sum)
    .def("distance", &Class::Distance, "Calc distance to the given point")
    .def("length", &Class::Length)
    .def("squared_length", &Class::SquaredLength)
    .def("normalize", &Class::Normalize)
    .def("normalized", &Class::Normalized)
    .def("round", &Class::Round)
    .def("rounded", &Class::Rounded)
    .def("set", &Class::Set)
    .def("dot", &Class::Dot)
    .def("abs", &Class::Abs)
    .def("abs_dot", &Class::AbsDot)
    .def("max", py::overload_cast<const Class&>(&Class::Max))
    .def("max", py::overload_cast<>(&Class::Max, py::const_))
    .def("min", py::overload_cast<const Class&>(&Class::Min))
    .def("min", py::overload_cast<>(&Class::Min, py::const_))
    .def(py::self + py::self)
    .def(py::self += py::self)
    .def(py::self + T())
    .def(py::self += T())
    .def(py::self * py::self)
    .def(py::self *= py::self)
    .def(py::self * T())
    .def(py::self *= T())
    .def(py::self - py::self)
    .def(py::self -= py::self)
    .def(py::self - T())
    .def(py::self -= T())
    .def(py::self / py::self)
    .def(py::self /= py::self)
    .def(py::self / T())
    .def(py::self /= T())
    .def(py::self != py::self)
    .def(py::self == py::self)
    .def(-py::self)
    .def("equal", &Class::Equal)
    .def("is_finite", &Class::IsFinite)
    .def("correct", &Class::Correct)
    .def("x", py::overload_cast<>(&Class::X))
    .def("y", py::overload_cast<>(&Class::Y))
    .def("x", py::overload_cast<const T&>(&Class::X))
    .def("y", py::overload_cast<const T&>(&Class::Y))
    .def_readonly_static("ZERO", &Class::Zero)
    .def_readonly_static("ONE", &Class::One)
    .def_readonly_static("NAN", &Class::NaN)
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a)
    .def("__getitem__", py::overload_cast<const std::size_t>(&Class::operator[], py::const_))
    .def("__setitem__", [](Class* vec, unsigned index, T val) { (*vec)[index] = val; })
    .def("__str__", [](const Class &si) {
      std::stringstream stream;
      stream << si;
      return stream.str();
    })
    .def("__repr__", [](const Class &si) {
      std::stringstream stream;
      stream << si;
      return stream.str();
    });
}

}  // namespace python
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_PYTHON__SERVER_CONFIG_HPP_
