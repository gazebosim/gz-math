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
#include <ignition/math/Frame.hh>
#include <ignition/math/FrameGraph.hh>

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(FrameGraphTest, Constructor)
{
  FrameGraph graph(
      // The frames
      {{"a", Pose3d::Zero},
       {"b", Pose3d::Zero},
       {"c", Pose3d::Zero} },
      // The edge
      {{"a", "b"},
       {"a", "c"},
       {"c", "b"}});

  FrameWeakPtr node = graph.Node("a");
  EXPECT_FALSE(node.expired());
  EXPECT_TRUE(node.lock() != NULL);
  EXPECT_EQ(node.lock()->Name(), "a");
}

/////////////////////////////////////////////////
TEST(FrameGraphTest, Path)
{
  FrameGraph graph(
      // The frames
      {{"a", Pose3d(0, 0, 0, 0, 0, 0)},
       {"b", Pose3d(1, 0, 0, 0, 0, 0)},
       {"c", Pose3d(0, 1, 0, 0, 0, 0)},
       {"d", Pose3d(0, 0, 1, 0, 0, 0)},
       {"e", Pose3d(0, 0, 0, 1, 0, 0)}
       },
      // The edge
      {{"a", "b"},
       {"b", "c"},
       {"b", "e"},
       {"c", "d"},
       {"e", "d"}
       });

  std::cout << graph << std::endl;

  std::list<FrameWeakPtr> path;
  graph.Path("a", "d", path);
  EXPECT_EQ(path.size(), 4);
  auto it = path.begin();

  EXPECT_EQ((it++)->lock()->Name(), "a");
  EXPECT_EQ((it++)->lock()->Name(), "b");
  EXPECT_EQ((it++)->lock()->Name(), "e");
  EXPECT_EQ((it)->lock()->Name(), "d");

  Pose3d transform = graph.Transform("a", "d");
  EXPECT_EQ(transform, Pose3d(1, 0, 1, 1, 0, 0));
}
