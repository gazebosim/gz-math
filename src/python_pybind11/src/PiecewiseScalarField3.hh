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

#ifndef GZ_MATH_PYTHON__PIECEWISE_SCALAR_FIELD3_HH_
#define GZ_MATH_PYTHON__PIECEWISE_SCALAR_FIELD3_HH_

#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/math/AdditivelySeparableScalarField3.hh>
#include <gz/math/PiecewiseScalarField3.hh>
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
/// Help define a pybind11 wrapper for a gz::math::PiecewiseScalarField3
/// whose pieces are polynomial-backed
/// gz::math::AdditivelySeparableScalarField3 fields.
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void helpDefineMathPiecewiseScalarField3(
    py::module &m, const std::string &typestr)
{
  using FieldT = gz::math::AdditivelySeparableScalarField3<
      gz::math::Polynomial3<T>, T>;
  using Class = gz::math::PiecewiseScalarField3<FieldT, T>;
  using Piece = typename Class::Piece;
  auto toString = [](const Class &field)
  {
    std::stringstream stream;
    stream << field;
    return stream.str();
  };

  py::class_<Class> cls(m, typestr.c_str(), py::dynamic_attr());

  py::class_<Piece>(cls, "Piece")
    .def(py::init([](const Region3<T> &_region, const FieldT &_field)
         {
           return Piece{_region, _field};
         }),
         "region"_a, "field"_a)
    .def_readwrite("region", &Piece::region,
                   "Region in which the scalar field is defined")
    .def_readwrite("field", &Piece::field,
                   "Scalar field defined in the region");

  cls
    .def(py::init<>())
    .def(py::init<const std::vector<Piece> &>(), "pieces"_a)
    .def_static("throughout", &Class::Throughout,
                "Define piecewise scalar field as `_field` "
                "throughout R^3 space")
    .def("evaluate", &Class::Evaluate,
         "Evaluate the piecewise scalar field at `_p`")
    .def("minimum",
         [](const Class &self, Vector3<T> &_pMin)
         {
           T result = self.Minimum(_pMin);
           return std::make_tuple(result, _pMin);
         },
         "Compute the piecewise scalar field minimum")
    .def("minimum",
         py::overload_cast<>(&Class::Minimum, py::const_),
         "Compute the piecewise scalar field minimum")
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

/// Define pybind11 wrappers for gz::math::PiecewiseScalarField3 types
/// with polynomial-backed gz::math::AdditivelySeparableScalarField3 pieces.
/**
 * \param[in] module a pybind11 module to add the definitions to
 * \param[in] typestr base name of the types used by Python
 */
void defineMathPiecewiseScalarField3(
    py::module &m, const std::string &typestr);

}  // namespace python
}  // namespace math
}  // namespace gz

#endif  // GZ_MATH_PYTHON__PIECEWISE_SCALAR_FIELD3_HH_
