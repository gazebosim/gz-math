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
TEST(FrameGraphTest, ConstructTest)
{
  // store result of various calls
  bool r;

  // frameGraph comes with a built-in "world" frame
  FrameGraph frameGraph;

  Pose3d px1(1, 0, 0, 0, 0, 0);

  // this path is not fully qualified
  EXPECT_FALSE(frameGraph.AddFrame("x1", px1, "/world"));
  // this path's parent is incorrect
  EXPECT_FALSE(frameGraph.AddFrame("x1", px1, "world"));
  // this path as an undefined frame
  EXPECT_FALSE(frameGraph.AddFrame("/world/unknown/x1", px1, "/world"));

  // this path adds x1 to the built in "/world" frame
  EXPECT_TRUE(frameGraph.AddFrame("/world/x1", px1, "/world"));
std::cerr << "sdfsdf\n" ;
  Pose3d p;
  EXPECT_TRUE(frameGraph.Pose("/world/x1", "/world", p));
  EXPECT_TRUE(p == px1);

/*

  Pose3d px2(0, 1, 0, 0, 0, 0);
  EXPECT_TRUE(frameGraph.AddFrame("/world/xx1", px2, "x1"));


  std::cout << "POSE x1::world " << p << std::endl;
  EXPECT_EQ(1, p.Pos().X());

  r = frameGraph.Pose("y1", "world", p);
  EXPECT_TRUE(r);

  std::cout << "POSE y1::world " << p << std::endl;
  EXPECT_EQ(1, p.Pos().Y());
  EXPECT_EQ(1, p.Pos().X());


  Pose3d px = px2 + px1;
  std::cout << "POSE px: " << px << std::endl;
*/
}

//  EXPECT_EQ(2, 42);
//  EXPECT_DOUBLE_EQ(frustum.AspectRatio(), 1.3434);


/////////////////////////////////////////////////
TEST(FrameGraphTest, Poseing)
{
  std::cout << "===== POSE TEST =====" << std::endl;
}
