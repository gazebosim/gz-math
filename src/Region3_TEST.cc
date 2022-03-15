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

#include "ignition/math/Region3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Region3Test, DefaultConstructor)
{
  const math::Region3d region;
  EXPECT_TRUE(region.Ix().Empty());
  EXPECT_TRUE(region.Iy().Empty());
  EXPECT_TRUE(region.Iz().Empty());
}

/////////////////////////////////////////////////
TEST(Region3Test, Constructor)
{
  const math::Region3d region(
      math::Intervald::Open(0., 1.),
      math::Intervald::Closed(-1., 1.),
      math::Intervald::Open(-1., 0.));
  EXPECT_EQ(region.Ix(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(region.Iy(), math::Intervald::Closed(-1., 1.));
  EXPECT_EQ(region.Iz(), math::Intervald::Open(-1., 0.));
}

/////////////////////////////////////////////////
TEST(Region3Test, ConstructionHelpers)
{
  const math::Region3d openRegion =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_EQ(openRegion.Ix(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(openRegion.Iy(), math::Intervald::Open(0., 1.));
  EXPECT_EQ(openRegion.Iz(), math::Intervald::Open(0., 1.));
  const math::Region3d closedRegion =
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
  EXPECT_EQ(closedRegion.Ix(), math::Intervald::Closed(0., 1.));
  EXPECT_EQ(closedRegion.Iy(), math::Intervald::Closed(0., 1.));
  EXPECT_EQ(closedRegion.Iz(), math::Intervald::Closed(0., 1.));
}

/////////////////////////////////////////////////
TEST(Region3Test, EmptyCheck)
{
  EXPECT_FALSE(math::Region3d::Open(0., 0., 0., 1., 1., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 0., 0., 0.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 0., 1., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 1., 0., 1.).Empty());
  EXPECT_TRUE(math::Region3d::Open(0., 0., 0., 1., 1., 0.).Empty());
  EXPECT_FALSE(math::Region3d::Closed(0., 0., 0., 0., 0., 0.).Empty());
  EXPECT_TRUE(math::Region3d::Closed(1., 1., 1., 0., 0., 0.).Empty());
}

/////////////////////////////////////////////////
TEST(Region3Test, MembershipCheck)
{
  const math::Region3d openRegion =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_FALSE(openRegion.Contains(math::Vector3d(0., 0., 0.)));
  EXPECT_TRUE(openRegion.Contains(math::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_FALSE(openRegion.Contains(math::Vector3d(1., 1., 1.)));

  const math::Region3d closedRegion =
      math::Region3d::Closed(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(0., 0., 0.)));
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_TRUE(closedRegion.Contains(math::Vector3d(1., 1., 1.)));

  EXPECT_TRUE(closedRegion.Contains(openRegion));
  EXPECT_FALSE(openRegion.Contains(closedRegion));
}

/////////////////////////////////////////////////
TEST(Region3Test, IntersectionCheck)
{
  const math::Region3d region =
      math::Region3d::Open(0., 0., 0., 1., 1., 1.);
  EXPECT_TRUE(region.Intersects(
      math::Region3d::Open(0.5, 0.5, 0.5, 1.5, 1.5, 1.5)));
  EXPECT_TRUE(region.Intersects(
      math::Region3d::Open(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)));
  EXPECT_FALSE(region.Intersects(
      math::Region3d::Open(1., 1., 1., 2., 2., 2.)));
  EXPECT_FALSE(region.Intersects(
      math::Region3d::Open(-1., -1., -1., 0., 0., 0.)));
}
