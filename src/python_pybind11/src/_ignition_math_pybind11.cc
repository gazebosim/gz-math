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

#include <pybind11/pybind11.h>

#include "vector2.hh"
#include "vector3.hh"
#include "vector4.hh"

namespace py = pybind11;

PYBIND11_MODULE(math, m)
{
  m.doc() = "Ignition Math Python Library.";

  ignition::math::python::define_math_vector2<double>(m, "Vector2d");
  ignition::math::python::define_math_vector2<int>(m, "Vector2i");
  ignition::math::python::define_math_vector2<float>(m, "Vector2f");

  ignition::math::python::define_math_vector3<double>(m, "Vector3d");
  ignition::math::python::define_math_vector3<int>(m, "Vector3i");
  ignition::math::python::define_math_vector3<float>(m, "Vector3f");

  ignition::math::python::define_math_vector4<double>(m, "Vector4d");
  ignition::math::python::define_math_vector4<int>(m, "Vector4i");
  ignition::math::python::define_math_vector4<float>(m, "Vector4f");
}
