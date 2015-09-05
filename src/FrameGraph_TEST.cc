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
TEST(FrameGraphTest, AbsolutePaths)
{
  // store result of various calls
  bool r;

  // frameGraph comes with a built-in "world" frame
  FrameGraph frameGraph;

  Pose3d pa(1, 0, 0, 0, 0, 0);
  // this path is not fully qualified
  EXPECT_FALSE(frameGraph.AddFrame("x", pa, "/world"));
  // this path's parent is incorrect ("world has no /")
  EXPECT_FALSE(frameGraph.AddFrame("x", pa, "world"));
  // this path as an undefined "unknown" frame
  EXPECT_FALSE(frameGraph.AddFrame("/world/unknown/x", pa, "/world"));

  Pose3d p;
  EXPECT_FALSE(frameGraph.Pose("/world/x", "/world", p));

  // this path adds a to the built in "/world" frame
  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "/world"));
  Pose3d a2w;
  EXPECT_TRUE(frameGraph.Pose("/world/a", "/world", a2w));
  EXPECT_EQ(pa, a2w);

  // error: x does not exist
  EXPECT_FALSE(frameGraph.Pose("/world/a", "/world/x", p));

  Pose3d pb(0, 1, 0, 0, 0, 0);
  EXPECT_TRUE(frameGraph.AddFrame("/world/b", pb, "/world"));
  Pose3d w2b;

  // Tests using relative paths
  EXPECT_TRUE(frameGraph.Pose("/world/b", "..", w2b));
  EXPECT_EQ(pb, w2b);


  Pose3d b2a, b2a2;
  EXPECT_TRUE(frameGraph.Pose("/world/a", "/world/b", b2a));
  EXPECT_TRUE(frameGraph.Pose("/world/a", "../b", b2a2));
  EXPECT_EQ(b2a, b2a2);

}

/////////////////////////////////////////////////
TEST(FrameGraphTest, RelativePaths)
{
  //             world
  //              |
  //              a
  //              |
  //          --------
  //          |      |
  //          aa     ab
  //          |
  //         aaa
  FrameGraph frameGraph;
  Pose3d pa(100, 0, 0, 0, 0, 0);
  EXPECT_TRUE (frameGraph.AddFrame("/world/a", pa, "."));
  EXPECT_FALSE(frameGraph.AddFrame("/world/a", pa, ".."));
/*
  EXPECT_FALSE(frameGraph.AddFrame("/world", "a", pa, "."));

  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "."));
  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "."));
  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "."));
  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "."));
*/
}


/////////////////////////////////////////////////
TEST(FrameGraphTest, SimplePose)
{
  FrameGraph frameGraph;

  std::cout << "===== POSE TEST =====" << std::endl;

  Pose3d pa(1, 0, 0, 0, 0, 0);
  EXPECT_TRUE(frameGraph.AddFrame("/world/a", pa, "/world"));

  Pose3d r;
  EXPECT_TRUE(frameGraph.Pose("/world/a", "/world", r));
  EXPECT_EQ(pa, r);

  Pose3d *p = frameGraph.FramePose("/world/a");
  EXPECT_EQ(pa, *p);

  Pose3d dz(0,0,1,0,0,0);
  *p += dz;
}


