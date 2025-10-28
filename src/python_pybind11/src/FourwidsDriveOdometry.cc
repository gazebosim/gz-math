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
#include <string>

#include <gz/math/FourwidsDriveOdometry.hh>

#include "FourwidsDriveOdometry.hh"

namespace gz
{
namespace math
{
namespace python
{
void defineMathFourwidsDriveOdometry(py::module &m, const std::string &typestr)
{
  using Class = gz::math::FourwidsDriveOdometry;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol())
  .def(py::init<size_t>(), py::arg("_windowSize") = 10)
  .def("init", &Class::Init, "Initialize the odometry")
  .def("initialized", &Class::Initialized, "Get whether Init has been called.")
  .def("update",
       &Class::Update,
       "Updates the odometry class with latest wheels and "
       "steerings position")
  .def("heading", &Class::Heading, "Get the heading.")
  .def("x", &Class::X, "Get the X position.")
  .def("y", &Class::Y, "Get the Y position.")
  .def("linear_velocity",
       &Class::LinearVelocity,
       "Get the linear velocity.")
  .def("lateral_velocity",
       &Class::LateralVelocity,
       "Get the lateral velocity.")
  .def("angular_velocity",
       &Class::AngularVelocity,
       "Get the angular velocity.")
  .def("set_wheel_params",
       &Class::SetWheelParams,
       "Set the wheel parameters including the separation, base and radius.")
  .def("set_velocity_rolling_window_size",
       &Class::SetVelocityRollingWindowSize,
       "Set the velocity rolling window size.")
  .def("wheel_separation", &Class::WheelSeparation, "Get the wheel separation")
  .def("wheel_base", &Class::WheelBase, "Get the wheel base")
  .def("wheel_radius",
       &Class::WheelRadius,
       "Get the wheel radius");
}
}  // namespace python
}  // namespace math
}  // namespace gz
