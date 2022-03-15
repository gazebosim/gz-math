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
#include <gtest/gtest.h>

#include "ignition/math/AdditivelySeparableScalarField3.hh"
#include "ignition/math/PiecewiseScalarField3.hh"
#include "ignition/math/Polynomial3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(PiecewiseScalarField3Test, Evaluate)
{
  using ScalarField3dT = std::function<double(const math::Vector3d&)>;
  using PiecewiseScalarField3dT = math::PiecewiseScalarField3d<ScalarField3dT>;
  {
    const PiecewiseScalarField3dT F;
    EXPECT_TRUE(std::isnan(F(math::Vector3d::Zero)));
  }
  {
    const PiecewiseScalarField3dT F =
        PiecewiseScalarField3dT::Throughout(
            [](const math::Vector3d& v) { return v.X(); });
    EXPECT_DOUBLE_EQ(F(math::Vector3d::Zero), 0.);
    EXPECT_DOUBLE_EQ(F(math::Vector3d(0.5, 0.5, 0.5)), 0.5);
    EXPECT_DOUBLE_EQ(F(math::Vector3d::One), 1.);
    EXPECT_DOUBLE_EQ(F(math::Vector3d::UnitX), 1.);
  }
  {
    const math::Region3d region0 =
        math::Region3d::Open(0., 0., 0., 1., 1., 1.);
    auto field0 = [](const math::Vector3d& v) { return v.X(); };
    const math::Region3d region1 =
        math::Region3d::Open(-1., -1., -1., 0., 0., 0.);
    auto field1 = [](const math::Vector3d& v) { return v.Y(); };
    const PiecewiseScalarField3dT F({
        {region0, field0}, {region1, field1}});
    EXPECT_DOUBLE_EQ(F(math::Vector3d(0.5, 0.25, 0.5)), 0.5);
    EXPECT_DOUBLE_EQ(F(math::Vector3d(-0.5, -0.25, -0.5)), -0.25);
    EXPECT_TRUE(std::isnan(F(math::Vector3d(0.5, -0.25, 0.5))));
    EXPECT_TRUE(std::isnan(F(math::Vector3d(-0.5, 0.25, -0.5))));
  }
}

/////////////////////////////////////////////////
TEST(PiecewiseScalarField3Test, Minimum)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  using PiecewiseScalarField3dT =
      math::PiecewiseScalarField3d<AdditivelySeparableScalarField3dT>;
  {
    const PiecewiseScalarField3dT F =
        PiecewiseScalarField3dT::Throughout(
            AdditivelySeparableScalarField3dT(
                1.,
                math::Polynomial3d::Constant(0.),
                math::Polynomial3d::Constant(1.),
                math::Polynomial3d::Constant(0.)));
    EXPECT_DOUBLE_EQ(F.Minimum(), 1.);
  }
  {
    const math::Region3d region0 =
        math::Region3d::Open(0., 0., 0., 1., 1., 1.);
    const AdditivelySeparableScalarField3dT field0(
        1.,
        math::Polynomial3d(
            math::Vector4d(0., 1., 0., 0.)),
        math::Polynomial3d::Constant(0.),
        math::Polynomial3d::Constant(0.));
    const math::Region3d region1 =
        math::Region3d::Open(-1., -1., -1., 0., 0., 0.);
    const AdditivelySeparableScalarField3dT field1(
        1.,
        math::Polynomial3d::Constant(0.),
        math::Polynomial3d(
            math::Vector4d(0., 1., 0., 1.)),
        math::Polynomial3d::Constant(0.));
    const PiecewiseScalarField3dT F({
        {region0, field0}, {region1, field1}});
    EXPECT_DOUBLE_EQ(F.Minimum(), 0.);
  }
}
