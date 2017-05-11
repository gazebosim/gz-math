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

#include "ignition/math/Graph.hh"
#include "ignition/math/GraphAlgorithms.hh"

using namespace ignition;
using namespace math;

// Define a test fixture class template.
template <class T>
class GraphTestFixture : public testing::Test
{
};

// The list of graphs we want to test.
using GraphTypes = ::testing::Types<DirectedGraph<int, double>,
                                    UndirectedGraph<int, double>>;
TYPED_TEST_CASE(GraphTestFixture, GraphTypes);

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, UniformInitialization)
{
  TypeParam graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
  });

  // Verify the vertices.
  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  for (int i = 0; i < 3; ++i)
  {
    ASSERT_NE(vertices.find(i), vertices.end());
    auto v = vertices.at(i).get();
    EXPECT_EQ(v.Name(), std::to_string(i));
    EXPECT_EQ(v.Id(), i);
    EXPECT_EQ(v.Data(), i);
  }

  // Verify the edges.
  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 2u);
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, VertexFromId)
{
  TypeParam graph;

  // Create some vertices.
  auto v0 = graph.AddVertex("0", 0);
  EXPECT_EQ(v0.Name(), "0");
  auto v1 = graph.AddVertex("1", 1);
  auto v2 = graph.AddVertex("2", 2);

  auto v = graph.VertexFromId(v0.Id());
  EXPECT_EQ(v.Id(), v0.Id());

  // Id not found.
  v = graph.VertexFromId(-500);
  EXPECT_EQ(v.Id(), kNullId);
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, Vertices)
{
  TypeParam graph(
  {
    {{"0", 10, 0}, {"1", 20, 1}, {"2", 30, 2}},
    {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
  });

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  // Check that the vertex Ids start from 0.
  EXPECT_NE(vertices.find(0), vertices.end());
  EXPECT_NE(vertices.find(1), vertices.end());
  EXPECT_NE(vertices.find(2), vertices.end());

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 0:
      {
        EXPECT_EQ(vertex.Name(), "0");
        EXPECT_EQ(vertex.Data(), 10);
        break;
      }
      case 1:
      {
        EXPECT_EQ(vertex.Name(), "1");
        EXPECT_EQ(vertex.Data(), 20);
        break;
      }
      case 2:
      {
        EXPECT_EQ(vertex.Name(), "2");
        EXPECT_EQ(vertex.Data(), 30);
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, VerticesNames)
{
  // Create a few vertices with two of them sharing the same name.
  TypeParam graph(
  {
    {{"vertex_0", 0}, {"vertex_1", 1}, {"common", 2}, {"common", 3}},
    {}
  });

  auto vertices = graph.Vertices("common");
  EXPECT_EQ(vertices.size(), 2u);

  // Check the Ids.
  EXPECT_NE(vertices.find(2), vertices.end());
  EXPECT_NE(vertices.find(3), vertices.end());

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 2:
      {
        EXPECT_EQ(vertex.Name(), "common");
        EXPECT_EQ(vertex.Data(), 2);
        break;
      }
      case 3:
      {
        EXPECT_EQ(vertex.Name(), "common");
        EXPECT_EQ(vertex.Data(), 3);
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, Empty)
{
  TypeParam graph;

  EXPECT_TRUE(graph.Empty());

  // Create a vertex.
  auto v0 = graph.AddVertex("0", 0);
  ASSERT_TRUE(v0.Valid());
  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, AddVertex)
{
  TypeParam graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex("0", 0);
  EXPECT_TRUE(v0.Id() != kNullId);
  auto v1 = graph.AddVertex("1", 1);
  EXPECT_TRUE(v1.Id() != kNullId);
  auto v2 = graph.AddVertex("2", 2);
  EXPECT_TRUE(v2.Id() != kNullId);

  // Create a vertex with Id.
  auto v3 = graph.AddVertex("3", 5, 3);
  EXPECT_EQ(v3.Id(), 3);
  EXPECT_EQ(v3.Data(), 5);
  EXPECT_EQ(v3.Name(), "3");

  // Create a vertex with an already used Id.
  auto v4 = graph.AddVertex("3", 0, 3);
  ASSERT_TRUE(v4.Id() == kNullId);

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 4u);
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, BFS)
{
  TypeParam graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  auto res = BFS(graph, 0);
  std::vector<VertexId> expected = {0, 1, 2, 4, 3, 5, 6};
  EXPECT_EQ(res, expected);
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, Dijkstra)
{
  TypeParam graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  auto res = dijkstra(graph, 0, 5);
  std::vector<VertexId> expected = {0, 1, 5};
  EXPECT_EQ(res, expected);
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, DijkstraWeights)
{
  TypeParam graph;
  int kNumVertices = 10;
  for (int i = 0; i < kNumVertices; ++i)
    graph.AddVertex(std::to_string(i), i);

  for (int i = 0; i < kNumVertices; ++i)
    for (int j = i; j < kNumVertices; ++j)
    {
      if (i == 0 && j == 5)
        graph.AddEdge({i, j}, 2.0, 10.0);
      else
        graph.AddEdge({i, j}, 2.0);
    }

  VertexId from = 0;
  VertexId to = 5;
  auto res = dijkstra(graph, from, to);
  std::vector<VertexId> expected = {0, 1, 5};
  EXPECT_EQ(res, expected);
}
