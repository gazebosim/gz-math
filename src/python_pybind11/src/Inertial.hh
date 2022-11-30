/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_MATH_PYTHON__INERTIAL_HH_
#define GZ_MATH_PYTHON__INERTIAL_HH_

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <gz/math/Inertial.hh>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ignition
{
namespace math
{
namespace python
{
/// Define a pybind11 wrapper for an gz::math::Inertial
/**
 * \param[in] module a pybind11 module to add the definition to
 * \param[in] typestr name of the type used by Python
 */
template<typename T>
void defineMathInertial(py::module &m, const std::string &typestr)
{

  using Class = gz::math::Inertial<T>;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
    .def(py::init<>())
    .def(py::init<const gz::math::MassMatrix3<T>&,
                  const gz::math::Pose3<T>&>())
    .def(py::init<const Class&>())
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self += py::self)
    .def(py::self + py::self)
    .def("set_mass_matrix",
         &Class::SetMassMatrix,
         py::arg("_m") = gz::math::MassMatrix3<T>(),
         py::arg("_tolerance") = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>,
         "Set the mass and inertia matrix.")
    .def("mass_matrix",
         &Class::MassMatrix,
         "Get the mass and inertia matrix.")
    .def("set_pose",
         &Class::SetPose,
         "Set the pose of the center of mass reference frame.")
    .def("pose",
         &Class::Pose,
         "Get the pose of the center of mass reference frame.")
    .def("moi",
         &Class::Moi,
         "Get the moment of inertia matrix computer about the body's "
         "center of mass and expressed in this Inertial object’s frame F.")
    .def("set_inertial_rotation",
         &Class::SetInertialRotation,
         "Set the inertial pose rotation without affecting the "
         "MOI in the base coordinate frame.")
    .def("set_mass_matrix_rotation",
         &Class::SetMassMatrixRotation,
         py::arg("_q") = gz::math::Quaternion<T>::Identity,
         py::arg("_tol") = 1e-6,
         "Set the MassMatrix rotation (eigenvectors of inertia matrix) "
         "without affecting the MOI in the base coordinate frame.")
    .def("__copy__", [](const Class &self) {
      return Class(self);
    })
    .def("__deepcopy__", [](const Class &self, py::dict) {
      return Class(self);
    }, "memo"_a);
}

}  // namespace python
}  // namespace math
}  // namespace ignition

#endif  // GZ_MATH_PYTHON__INERTIAL_HH_
