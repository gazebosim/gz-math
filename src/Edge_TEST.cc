/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <iostream>
#include <string>

#include "ignition/math/Edge.hh"
#include "ignition/math/Vertex.hh"

using namespace ignition;
using namespace math;

// Define a test fixture class template.
template <class T>
class EdgeTestFixture : public testing::Test
{
};

// The list of edges we want to test.
using EdgeTypes = ::testing::Types<DirectedEdge<int>,
                                   UndirectedEdge<int>>;
TYPED_TEST_CASE(EdgeTestFixture, EdgeTypes);

/////////////////////////////////////////////////
TYPED_TEST(EdgeTestFixture, Accessors)
{
  {
    EdgeId id = 1;
    double weight = 2.0;
    VertexId_A vertices = {0, 1};
    int data = 3;
    TypeParam edge(id, weight, vertices, data);

    EXPECT_EQ(edge.Id(), id);
    EXPECT_DOUBLE_EQ(edge.Weight(), weight);
    EXPECT_EQ(edge.Vertices(), vertices);
    EXPECT_EQ(edge.Data(), data);
    // Modify the data.
    edge.Data() += 1;
    EXPECT_EQ(edge.Data(), data + 1);
    EXPECT_TRUE(edge.Valid());
  }

  {
    EdgeId id = kNullId;
    double weight = 2.0;
    VertexId_A vertices = {0, 1};
    int data = 3;
    TypeParam edge(id, weight, vertices, data);

    EXPECT_EQ(edge.Id(), id);
    EXPECT_DOUBLE_EQ(edge.Weight(), weight);
    VertexId_A expectedVertices = {kNullId, kNullId};
    EXPECT_EQ(edge.Vertices(), expectedVertices);
    EXPECT_EQ(edge.Data(), data);
    // Modify the data.
    edge.Data() += 1;
    EXPECT_EQ(edge.Data(), data + 1);
    EXPECT_FALSE(edge.Valid());
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionDirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_A vertices = {0, 1};
  int data = 3;
  DirectedEdge<int> edge(id, weight, vertices, data);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -> 1 [label=2];\n";
  EXPECT_EQ(output.str(), expectedOutput);
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionUndirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_A vertices = {0, 1};
  int data = 3;
  UndirectedEdge<int> edge(id, weight, vertices, data);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -- 1 [label=2];\n";
  EXPECT_EQ(output.str(), expectedOutput);
}
