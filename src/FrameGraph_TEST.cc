/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/math/Helpers.hh"
#include "ignition/math/FrameGraph.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(FrameGraphTest, Constructor)
{
  FrameGraph frameGraph;

  EXPECT_EQ(0, 0);
  EXPECT_EQ(1, 1);
  EXPECT_EQ(2, 42);
}

//  EXPECT_DOUBLE_EQ(frustum.AspectRatio(), 1.3434);


/////////////////////////////////////////////////
TEST(FrameGraphTest, Poseur)
{
  std::cout << "OUT!" << std::endl;
}
