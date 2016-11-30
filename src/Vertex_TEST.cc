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
#include <ignition/math/Vertex.hh>

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(Vertex, Constructor)
{
  Vertex<double> vertex;
  EXPECT_TRUE(vertex.Name().empty());
  EXPECT_DOUBLE_EQ(vertex.Data(), 0.0);

  Vertex<double> vertex2("test");
  EXPECT_EQ(vertex2.Name(), "test");
  double value = vertex2.Data();
  EXPECT_DOUBLE_EQ(value, 0.0);
  vertex2.SetName("another");
  EXPECT_EQ(vertex2.Name(), "another");

  Vertex<Pose3d> vertex3("test", Pose3d(0, 1, 2, 3, 4, 5, 6));
  EXPECT_EQ(vertex3.Name(), "test");
  EXPECT_EQ(vertex3.Data(), Pose3d(0, 1, 2, 3, 4, 5, 6));
  vertex3.SetData(Pose3d::Zero);
  EXPECT_EQ(vertex3.Data(), Pose3d::Zero);
}

/////////////////////////////////////////////////
TEST(Vertex, OutputStream)
{
  Vertex<Pose3d> vertex("test", Pose3d(0, 1, 2, 0.1, 0.2, 0.3));
  std::ostringstream stream;
  stream << vertex;
  EXPECT_EQ(stream.str(), "test = 0 1 2 0.1 0.2 0.3");
}
