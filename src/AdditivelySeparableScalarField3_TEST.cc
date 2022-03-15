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
#include <functional>
#include <ostream>

#include "ignition/math/AdditivelySeparableScalarField3.hh"
#include "ignition/math/Polynomial3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Evaluate)
{
  using ScalarFunctionT = std::function<double(double)>;
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<ScalarFunctionT>;
  const double k = 1.;
  auto f = [](double x) { return x; };
  const AdditivelySeparableScalarField3dT F(k, f, f, f);
  EXPECT_DOUBLE_EQ(F(math::Vector3d::Zero), 0.);
  EXPECT_DOUBLE_EQ(F(math::Vector3d::One), 3.);
  EXPECT_DOUBLE_EQ(F(math::Vector3d::UnitX), 1.);
  EXPECT_DOUBLE_EQ(F(math::Vector3d::UnitY), 1.);
  EXPECT_DOUBLE_EQ(F(math::Vector3d::UnitZ), 1.);
  const math::Vector3d INF_V(
      math::INF_D, math::INF_D, math::INF_D);
  EXPECT_DOUBLE_EQ(F(INF_V), math::INF_D);
}

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Minimum)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  constexpr double k = 1. / 3.;
  const math::Polynomial3d p(math::Vector4d(0., 1., 1., 1));
  const AdditivelySeparableScalarField3dT F(k, p, p, p);
  std::cout << F << std::endl;
  {
    const math::Region3d region =
        math::Region3d::Open(0., 0., 0., 0., 0., 0.);
    math::Vector3d pMin = math::Vector3d::Zero;
    EXPECT_TRUE(std::isnan(F.Minimum(region)));
    EXPECT_TRUE(std::isnan(F.Minimum(region, pMin)));
    EXPECT_TRUE(std::isnan(pMin.X()));
    EXPECT_TRUE(std::isnan(pMin.Y()));
    EXPECT_TRUE(std::isnan(pMin.Z()));
  }
  {
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(F.Minimum(), 0.75);
    EXPECT_DOUBLE_EQ(F.Minimum(pMin), 0.75);
    EXPECT_EQ(pMin, -0.5 * math::Vector3d::One);
  }
  {
    const math::Region3d region =
        math::Region3d::Open(1., 1., 1., 2., 2., 2.);
    math::Vector3d pMin = math::Vector3d::NaN;
    EXPECT_DOUBLE_EQ(F.Minimum(region), 3.);
    EXPECT_DOUBLE_EQ(F.Minimum(region, pMin), 3.);
    EXPECT_EQ(pMin, math::Vector3d::One);
  }
}

/////////////////////////////////////////////////
TEST(AdditivelySeparableScalarField3Test, Stream)
{
  using AdditivelySeparableScalarField3dT =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  {
    constexpr double k = 1.;
    const math::Polynomial3d p(math::Vector4d(0., 1., 0., 1));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(k, p, p, p);
    EXPECT_EQ(os.str(), "[(x^2 + 1) + (y^2 + 1) + (z^2 + 1)]");
  }
  {
    constexpr double k = -1.;
    const math::Polynomial3d p(math::Vector4d(1., 0., 1., 0));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(k, p, p, p);
    EXPECT_EQ(os.str(), "-[(x^3 + x) + (y^3 + y) + (z^3 + z)]");
  }
  {
    constexpr double k = 3.;
    const math::Polynomial3d p(math::Vector4d(0., 1., 0., 0));
    const math::Polynomial3d q(math::Vector4d(-1., 0., 0., 0));
    const math::Polynomial3d r(math::Vector4d(0., 0., 0., 1));
    std::ostringstream os;
    os << AdditivelySeparableScalarField3dT(k, p, q, r);
    EXPECT_EQ(os.str(), "3 [(x^2) + (-y^3) + (1)]");
  }
}
