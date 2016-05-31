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
#include <ignition/math/FrameGraph.hh>

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(FrameGraphTest, Constructor)
{
  FrameGraph graph(
      // The frames
      {{"a", Pose3d::Zero},
       {"b", Pose3d::Nan},
       {"c", Pose3d(0, 1, 2, 3, 4, 5, 6)} },
      // The edge
      {{"a", "b"},
       {"a", "c"},
       {"c", "b"}});

  const Frame &nodeA = graph.Node("a");
  const Frame &nodeB = graph.Node("b");
  const Frame &nodeC = graph.Node("c");
  EXPECT_EQ(nodeA.Name(), "a");
  EXPECT_EQ(nodeB.Name(), "b");
  EXPECT_EQ(nodeC.Name(), "c");

  EXPECT_EQ(nodeA.Pose(), Pose3d::Zero);
  EXPECT_EQ(nodeB.Pose(), Pose3d::Nan);
  EXPECT_EQ(nodeC.Pose(), Pose3d(0, 1, 2, 3, 4, 5, 6));

  EXPECT_FALSE(nodeA == nodeB);
  EXPECT_FALSE(nodeA == nodeC);
  EXPECT_TRUE(nodeA != nodeB);
  EXPECT_TRUE(nodeA != nodeC);

  EXPECT_FALSE(nodeB == nodeA);
  EXPECT_FALSE(nodeB == nodeC);
  EXPECT_TRUE(nodeB != nodeA);
  EXPECT_TRUE(nodeB != nodeC);

  FrameGraph graph2;
  EXPECT_EQ(graph2.NodeCount(), 0);
  EXPECT_EQ(graph2.EdgeCount(), 0);
  EXPECT_EQ(graph2.Node("test"), Frame::Nan);

  graph2.AddNode("test", Pose3d::Zero);
  EXPECT_EQ(graph2.NodeCount(), 1);
  EXPECT_EQ(graph2.Node("test"), Frame("test", Pose3d::Zero));

  graph2.AddNode("test2", Pose3d::Zero);
  EXPECT_EQ(graph2.NodeCount(), 2);

  const Frame &newNode = graph2.AddNode(nodeA);
  EXPECT_EQ(graph2.NodeCount(), 3);

  EXPECT_TRUE(graph2.AddEdge("test", "test2"));
  EXPECT_FALSE(graph2.AddEdge("test", "not here"));

  graph.ClearEdges();
  EXPECT_TRUE(graph2.AddEdge(graph2.Node("test"), graph2.Node("test2")));
  EXPECT_FALSE(graph2.AddEdge(graph2.Node("test"), graph.Node("b")));
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
      // The edges
      {{"a", "b"},
       {"b", "c"},
       {"b", "e"},
       {"c", "d"},
       {"e", "d"}
       });

  std::cout << graph << std::endl;

  std::list<Frame> path;

  // No path with invalid start node
  EXPECT_FALSE(graph.Path("f", "d", path));
  EXPECT_TRUE(path.empty());

  // No path with invalid end node
  EXPECT_FALSE(graph.Path("a", "f", path));
  EXPECT_TRUE(path.empty());

  EXPECT_TRUE(graph.Path("a", "d", path));
  EXPECT_EQ(path.size(), 4);
  auto it = path.begin();

  EXPECT_EQ((*(it++)).Name(), "a");
  EXPECT_EQ((*(it++)).Name(), "b");
  EXPECT_EQ((*(it++)).Name(), "e");
  EXPECT_EQ((*(it)).Name(), "d");

  Pose3d transform = graph.Transform("a", "d");
  EXPECT_EQ(transform, Pose3d(1, 0, 1, 1, 0, 0));
}
