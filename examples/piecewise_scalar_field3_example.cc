/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
//! [complete]
#include <iostream>

#include <ignition/math/AdditivelySeparableScalarField3.hh>
#include <ignition/math/PiecewiseScalarField3.hh>
#include <ignition/math/Polynomial3.hh>

int main(int argc, char **argv)
{
  const double k = 1.;
  const ignition::math::Polynomial3d px(
      ignition::math::Vector4d(0., 1., 0., 1.));
  const ignition::math::Polynomial3d qy(
      ignition::math::Vector4d(1., 0., 1., 0.));
  const ignition::math::Polynomial3d rz(
      ignition::math::Vector4d(1., 0., 0., -1.));
  using AdditivelySeparableScalarField3dT =
      ignition::math::AdditivelySeparableScalarField3d<
        ignition::math::Polynomial3d>;
  using PiecewiseScalarField3dT =
      ignition::math::PiecewiseScalarField3d<
        AdditivelySeparableScalarField3dT>;
  const PiecewiseScalarField3dT P({
      {ignition::math::Region3d(  // x < 0 halfspace
          ignition::math::Intervald::Open(
              -ignition::math::INF_D, 0.),
          ignition::math::Intervald::Unbounded,
          ignition::math::Intervald::Unbounded),
       AdditivelySeparableScalarField3dT(k, px, qy, rz)},
      {ignition::math::Region3d(  // x >= 0 halfspace
          ignition::math::Intervald::LeftClosed(
              0., ignition::math::INF_D),
          ignition::math::Intervald::Unbounded,
          ignition::math::Intervald::Unbounded),
       AdditivelySeparableScalarField3dT(-k, px, qy, rz)}});

  // A printable piecewise scalar field.
  std::cout << "A piecewise scalar field in R^3 is made up of "
            << "several pieces e.g. P(x, y, z) = " << P << std::endl;

  // A piecewise scalar field can be evaluated.
  std::cout << "Evaluating P(x, y, z) at (1, 0, 0) yields "
            << P(ignition::math::Vector3d::UnitX)
            << std::endl;
  std::cout << "Evaluating P(x, y, z) at (-1, 0, 0) yields "
            << P(-ignition::math::Vector3d::UnitX)
            << std::endl;

  // A piecewise scalar field can be queried for its minimum
  // (provided the underlying scalar function allows it).
  std::cout << "The global minimum of P(x, y, z) is "
            << P.Minimum() << std::endl;
}
//! [complete]
