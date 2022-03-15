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

#include "ignition/math/Interval.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(IntervalTest, DefaultConstructor)
{
  const math::Intervald interval;
  EXPECT_DOUBLE_EQ(
      interval.LeftValue(),
      interval.RightValue());
}

/////////////////////////////////////////////////
TEST(IntervalTest, Constructor)
{
  constexpr bool kClosed = true;

  const math::Intervald interval(
      0., kClosed, 1., !kClosed);
  EXPECT_DOUBLE_EQ(interval.LeftValue(), 0.);
  EXPECT_TRUE(interval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(interval.RightValue(), 1.);
  EXPECT_FALSE(interval.IsRightClosed());
}

/////////////////////////////////////////////////
TEST(IntervalTest, ConstructionHelpers)
{
  const math::Intervald openInterval =
      math::Intervald::Open(0., 1.);
  EXPECT_DOUBLE_EQ(openInterval.LeftValue(), 0.);
  EXPECT_FALSE(openInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(openInterval.RightValue(), 1.);
  EXPECT_FALSE(openInterval.IsRightClosed());
  const math::Intervald closedInterval =
      math::Intervald::Closed(0., 1.);
  EXPECT_DOUBLE_EQ(closedInterval.LeftValue(), 0.);
  EXPECT_TRUE(closedInterval.IsLeftClosed());
  EXPECT_DOUBLE_EQ(closedInterval.RightValue(), 1.);
  EXPECT_TRUE(closedInterval.IsRightClosed());
}

/////////////////////////////////////////////////
TEST(IntervalTest, EmptyCheck)
{
  EXPECT_FALSE(math::Intervald::Open(0., 1.).Empty());
  EXPECT_TRUE(math::Intervald::Open(0., 0.).Empty());
  EXPECT_FALSE(math::Intervald::Closed(0., 0.).Empty());
  EXPECT_TRUE(math::Intervald::Closed(1., 0.).Empty());
}

/////////////////////////////////////////////////
TEST(IntervalTest, MembershipCheck)
{
  const math::Intervald openInterval =
      math::Intervald::Open(0., 1.);
  EXPECT_FALSE(openInterval.Contains(0.));
  EXPECT_TRUE(openInterval.Contains(0.5));
  EXPECT_FALSE(openInterval.Contains(1));

  EXPECT_TRUE(openInterval.Contains(openInterval));

  const math::Intervald closedInterval =
      math::Intervald::Closed(0., 1.);
  EXPECT_TRUE(closedInterval.Contains(0.));
  EXPECT_TRUE(closedInterval.Contains(0.5));
  EXPECT_TRUE(closedInterval.Contains(1));

  EXPECT_TRUE(closedInterval.Contains(closedInterval));
  EXPECT_FALSE(openInterval.Contains(closedInterval));
  EXPECT_TRUE(closedInterval.Contains(openInterval));

  const math::Intervald emptyInterval =
      math::Intervald::Open(0., 0.);
  EXPECT_FALSE(emptyInterval.Contains(0.));

  const math::Intervald degenerateInterval =
      math::Intervald::Closed(0., 0.);
  EXPECT_TRUE(degenerateInterval.Contains(0.));

  EXPECT_FALSE(closedInterval.Contains(emptyInterval));
  EXPECT_TRUE(closedInterval.Contains(degenerateInterval));
}

/////////////////////////////////////////////////
TEST(IntervalTest, IntersectionCheck)
{
  const math::Intervald interval =
      math::Intervald::Open(0., 1.);
  EXPECT_TRUE(interval.Intersects(math::Intervald::Open(0.5, 1.5)));
  EXPECT_TRUE(interval.Intersects(math::Intervald::Open(-0.5, 0.5)));
  EXPECT_FALSE(interval.Intersects(math::Intervald::Open(1., 2.)));
  EXPECT_FALSE(interval.Intersects(math::Intervald::Open(-1., 0.)));
  EXPECT_FALSE(interval.Intersects(math::Intervald::Open(0.5, 0.5)));
}

/////////////////////////////////////////////////
TEST(IntervalTest, Unbounded)
{
  EXPECT_FALSE(math::Intervald::Unbounded.Empty());
  EXPECT_FALSE(math::Intervald::Unbounded.IsLeftClosed());
  EXPECT_FALSE(math::Intervald::Unbounded.IsRightClosed());
}
