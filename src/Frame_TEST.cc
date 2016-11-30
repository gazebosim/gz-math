/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <ignition/math/Frame.hh>

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(Frame, Constructor)
{
  Frame frame;
  EXPECT_TRUE(frame.Name().empty());
  EXPECT_EQ(frame.Pose(), Pose3d::Zero);

  Frame frame2("test");
  EXPECT_EQ(frame2.Name(), "test");
  EXPECT_EQ(frame2.Pose(), Pose3d::Zero);
  frame2.SetName("another");
  EXPECT_EQ(frame2.Name(), "another");

  Frame frame3("test", Pose3d(0, 1, 2, 3, 4, 5, 6));
  EXPECT_EQ(frame3.Name(), "test");
  EXPECT_EQ(frame3.Pose(), Pose3d(0, 1, 2, 3, 4, 5, 6));
  frame3.SetPose(Pose3d::Zero);
  EXPECT_EQ(frame3.Pose(), Pose3d::Zero);

  EXPECT_FALSE(frame == frame2);
  EXPECT_TRUE(frame != frame2);
  EXPECT_FALSE(frame == frame3);
  EXPECT_TRUE(frame != frame3);
  EXPECT_FALSE(frame2 == frame3);
  EXPECT_TRUE(frame2 != frame3);

  Frame frame4(frame3);
  EXPECT_TRUE(frame4 == frame3);
  EXPECT_FALSE(frame4 != frame3);
}

/////////////////////////////////////////////////
TEST(Frame, OutputStream)
{
  Frame frame("test", Pose3d(0, 1, 2, 0.1, 0.2, 0.3));
  std::ostringstream stream;
  stream << frame;
  EXPECT_EQ(stream.str(), "test = 0 1 2 0.1 0.2 0.3");
}
