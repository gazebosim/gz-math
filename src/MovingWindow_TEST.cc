/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "ignition/math/movingwindow/AxisAlignedBoxWindow.hh"
#include "ignition/math/movingwindow/MovingWindow.hh"

using namespace ignition;
using namespace math;
using namespace movingwindow;

/////////////////////////////////////////////////
TEST(MovingWindowTest, ConstructorDefault)
{
  using WindowType = AxisAlignedBoxMovingWindow<AxisAlignedBox>;
  WindowType window{{}};
  auto res = window.Check();
  EXPECT_EQ(0ul, res.size());
}

/////////////////////////////////////////////////
TEST(MovingWindowTest, RegisterEntities)
{
  using WindowType = AxisAlignedBoxMovingWindow<AxisAlignedBox>;
  WindowType window{{-10, -10, 0, 10, 10, 10}};
  const std::size_t shapeId = 1;
  const bool res =
      window.RegisterEntity(shapeId, {-1, -1, -1, 1, 1, 1});
  EXPECT_TRUE(res);
  EXPECT_EQ(1ul, window.EntityCount());

  const bool res2 =
      window.RegisterEntity(shapeId, {-1, -1, -1, 1, 1, 1});
  EXPECT_FALSE(res2);

  const bool unregRes = window.UnregisterEntity(shapeId);
  EXPECT_TRUE(unregRes);
  EXPECT_EQ(0ul, window.EntityCount());
}

/////////////////////////////////////////////////
TEST(MovingWindowTest, TestWindowCheck)
{
  using WindowType = AxisAlignedBoxMovingWindow<AxisAlignedBox>;
  WindowType window{{-10, -10, 0, 10, 10, 10}};
  const std::size_t shapeId = 1;
  const bool res =
      window.RegisterEntity(shapeId, {-1, -1, -1, 1, 1, 1});
  ASSERT_TRUE(res);
  ASSERT_EQ(1ul, window.EntityCount());
  
  const auto checkRes = window.Check();
  ASSERT_GT(checkRes.size(), 0ul);
  EXPECT_EQ(shapeId, checkRes.begin()->id);
  EXPECT_EQ(EntityState::INSIDE, checkRes.begin()->state);
}

/////////////////////////////////////////////////
TEST(MovingWindowTest, TestShapePositionChange)
{
  using WindowType = AxisAlignedBoxMovingWindow<AxisAlignedBox>;
  WindowType window{{-10, -10, 0, 10, 10, 10}};
  const std::size_t shapeId = 1;
  const bool res =
      window.RegisterEntity(shapeId, {-1, -1, -1, 1, 1, 1});
  ASSERT_TRUE(res);
  
  const auto checkRes = window.Check();
  EXPECT_EQ(shapeId, checkRes.begin()->id);
  EXPECT_EQ(EntityState::INSIDE, checkRes.begin()->state);
  
  EXPECT_TRUE(window.SetEntityPose(shapeId, {100, 0, 0, 0, 0, 0}));
  const auto checkRes2 = window.Check();
  ASSERT_GT(checkRes2.size(), 0ul);
  EXPECT_EQ(shapeId, checkRes2.begin()->id);
  EXPECT_EQ(EntityState::OUTSIDE, checkRes2.begin()->state);
}
