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

#include "ignition/math/Polynomial3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Polynomial3Test, DefaultConstructor)
{
  const math::Polynomial3d poly;
  EXPECT_EQ(poly.Coeffs(), math::Vector4d::Zero);
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Constructor)
{
  const math::Polynomial3d poly(math::Vector4d::One);
  EXPECT_EQ(poly.Coeffs(), math::Vector4d::One);
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, ConstructionHelpers)
{
  const math::Polynomial3d poly = math::Polynomial3d::Constant(1.);
  EXPECT_EQ(poly.Coeffs(), math::Vector4d(0., 0., 0., 1.));
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Evaluate)
{
  {
    const math::Polynomial3d p =
        math::Polynomial3d::Constant(1.);
    EXPECT_DOUBLE_EQ(p(-1.), 1.);
    EXPECT_DOUBLE_EQ(p(0.), 1.);
    EXPECT_DOUBLE_EQ(p(1.), 1.);
    EXPECT_DOUBLE_EQ(p(math::INF_D), 1.);
  }
  {
    const math::Polynomial3d p(math::Vector4d::One);
    EXPECT_DOUBLE_EQ(p(-1.), 0.);
    EXPECT_DOUBLE_EQ(p(0.), 1.);
    EXPECT_DOUBLE_EQ(p(1.), 4.);
    EXPECT_DOUBLE_EQ(p(-math::INF_D), -math::INF_D);
  }
}

/////////////////////////////////////////////////
TEST(Polynomial3Test, Minimum)
{
  {
    const math::Polynomial3d p0 =
        math::Polynomial3d::Constant(1.);
    EXPECT_DOUBLE_EQ(p0.Minimum(), 1.);
  }
  {
    const math::Polynomial3d p1(
        math::Vector4d(0., 0., 1., 1.));
    EXPECT_DOUBLE_EQ(p1.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p1.Minimum(
        math::Intervald::Open(0., 1.)), 1.);
  }
  {
    const math::Polynomial3d p2(
        math::Vector4d(0., 1., 1., 1.));
    EXPECT_DOUBLE_EQ(p2.Minimum(), 0.75);
    EXPECT_DOUBLE_EQ(p2.Minimum(
        math::Intervald::Open(1., 2.)), 3.);
  }
  {
    const math::Polynomial3d p3(
        math::Vector4d(1., 1., 1., 1.));
    EXPECT_DOUBLE_EQ(p3.Minimum(), -math::INF_D);
    EXPECT_DOUBLE_EQ(p3.Minimum(
        math::Intervald::Open(-1., 1.)), 0.);
  }
}
