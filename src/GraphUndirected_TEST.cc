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
#include <algorithm>
#include <iostream>
#include <memory>

#include "ignition/math/GraphUndirected.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, UniformInitialization)
{
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
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
TEST(UndirectedGraphTest, VertexById)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  EXPECT_EQ(v0.Name(), "0");
  auto v1 = graph.AddVertex(1, "1");
  auto v2 = graph.AddVertex(2, "2");

  auto v = graph.VertexById(v0.Id());
  EXPECT_EQ(v.Id(), v0.Id());

  // Id not found.
  v = graph.VertexById(-500);
  EXPECT_EQ(v.Id(), kNullId);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, vertices)
{
  UndirectedGraph<int, double> graph(
  {
    {{0, "0"}, {1, "1"}, {2, "2"}},
    {}
  });

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  // Check that the vertices Ids start from 0.
  EXPECT_NE(vertices.find(0), vertices.end());
  EXPECT_NE(vertices.find(1), vertices.end());
  EXPECT_NE(vertices.find(2), vertices.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, VerticesNames)
{
  // Create a few vertices with two of them sharing the same name.
  UndirectedGraph<int, double> graph(
  {
    {{0, "vertex_0"}, {1, "vertex_1"}, {2, "common"}, {3, "common"}},
    {}
  });

  auto vertices = graph.Vertices("common");
  EXPECT_EQ(vertices.size(), 2);
  // Check the Ids.
  EXPECT_NE(vertices.find(2), vertices.end());
  EXPECT_NE(vertices.find(3), vertices.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Edges)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Check the Ids.
  EXPECT_NE(edges.find(0), edges.end());
  EXPECT_NE(edges.find(1), edges.end());
  EXPECT_NE(edges.find(2), edges.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Empty)
{
  UndirectedGraph<int, double> graph;

  EXPECT_TRUE(graph.Empty());

  // Create a vertex.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0.Valid());
  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Adjacents)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Try to get the adjacents of an inexistent vertex.
  auto adjacents = graph.Adjacents(kNullId);
  EXPECT_TRUE(adjacents.empty());

  adjacents = graph.Adjacents(0);
  EXPECT_EQ(adjacents.size(), 2u);
  EXPECT_NE(adjacents.find(1), adjacents.end());
  EXPECT_NE(adjacents.find(2), adjacents.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Incidents)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  auto incidents = graph.Incidents(0);
  EXPECT_EQ(incidents.size(), 2u);
  EXPECT_NE(incidents.find(2), incidents.end());
  EXPECT_NE(incidents.find(0), incidents.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, AddVertex)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  EXPECT_TRUE(v0.Id() != kNullId);
  auto v1 = graph.AddVertex(1, "1");
  EXPECT_TRUE(v1.Id() != kNullId);
  auto v2 = graph.AddVertex(2, "2");
  EXPECT_TRUE(v2.Id() != kNullId);

  // Create a vertex with Id.
  auto v3 = graph.AddVertex(5, "3", 3);
  EXPECT_EQ(v3.Id(), 3);
  EXPECT_EQ(v3.Data(), 5);
  EXPECT_EQ(v3.Name(), "3");

  // Create a vertex with an already used Id.
  auto v4 = graph.AddVertex(0, "3", 3);
  ASSERT_TRUE(v4.Id() == kNullId);

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 4u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, AddEdge)
{
  // Create a graph with three vertices.
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {}
  });

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge({0, 1}, 2.0);
  auto e1 = graph.AddEdge({1, 2}, 3.0);
  auto e2 = graph.AddEdge({2, 0}, 4.0);

  // Check the edge content.
  EXPECT_EQ(e0.Data(), 2.0);
  EXPECT_EQ(e1.Data(), 3.0);
  EXPECT_EQ(e2.Data(), 4.0);

  // Check that the edges point to the right vertices.
  EXPECT_NE(e0.Vertices().find(0), e0.Vertices().end());

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect tail.
  auto edge = graph.AddEdge({kNullId, 1}, 2.0);
  EXPECT_EQ(edge.Id(), kNullId);
  EXPECT_EQ(graph.Edges().size(), 3u);

  // Try to add an edge with an incorrect head.
  edge = graph.AddEdge({0, kNullId}, 2.0);
  EXPECT_EQ(edge.Id(), kNullId);
  EXPECT_EQ(graph.Edges().size(), 3u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveEdge)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Remove a nonexistent edge shouldn't cause any effect.
  EXPECT_FALSE(graph.RemoveEdge(kNullId));
  EXPECT_EQ(graph.Edges().size(), 3u);
  EXPECT_EQ(graph.Incidents(1).size(), 2u);

  // Remove the edge (v0-->v1)
  EXPECT_TRUE(graph.RemoveEdge(0));
  EXPECT_EQ(graph.Edges().size(), 2u);
  EXPECT_EQ(graph.Incidents(1).size(), 1);

  // Remove the edge (v1-->v2)
  EXPECT_TRUE(graph.RemoveEdge(1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  // Try to remove an edge that doesn't exist anymore.
  EXPECT_FALSE(graph.RemoveEdge(1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  // Remove the edge (v2-->v0)
  EXPECT_TRUE(graph.RemoveEdge(2));
  EXPECT_EQ(graph.Edges().size(), 0u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveVertex)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Remove a nonexistent vertex shouldn't cause any effect.
  EXPECT_FALSE(graph.RemoveVertex(kNullId));
  EXPECT_EQ(graph.Vertices().size(), 3u);
  EXPECT_EQ(graph.Adjacents(1).size(), 2u);

  // Remove vertex #2.
  EXPECT_TRUE(graph.RemoveVertex(2));
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);
  EXPECT_EQ(graph.Adjacents(1).size(), 1u);

  // Remove vertex #1.
  EXPECT_TRUE(graph.RemoveVertex(1));
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  // Remove vertex #0.
  EXPECT_TRUE(graph.RemoveVertex(0));
  EXPECT_TRUE(graph.Vertices().empty());
  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveVerticesWithName)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v3), (v3-->v0)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "v0", 0}, {1, "v1", 1}, {2, "common", 2}, {3, "common", 3}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 3}, 4.0}, {{3, 0}, 5.0}}
  });

  // Try to remove a node with a name that doesn't exist.
  EXPECT_EQ(graph.RemoveVertices("wrong_name"), 0);
  EXPECT_EQ(graph.Vertices().size(), 4u);
  EXPECT_EQ(graph.Adjacents(1).size(), 2u);

  // Remove two vertices at the same time.
  EXPECT_EQ(graph.RemoveVertices("common"), 2u);
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);
  EXPECT_EQ(graph.Adjacents(1).size(), 1u);

  // Remove vertex #1.
  EXPECT_EQ(graph.RemoveVertices("v1"), 1u);
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  // Remove vertex #0.
  EXPECT_EQ(graph.RemoveVertices("v0"), 1u);
  EXPECT_TRUE(graph.Vertices().empty());
  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, StreamInsertion)
{
  // Create a graph with 4 vertices and edges [(v0-->v1), (v1-->v2), (v2-->v3)]
  UndirectedGraph<int, double> graph(
  {
    {{0, "v0", 0}, {1, "v1", 1}, {2, "v2", 2}, {3, "v3", 3}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  std::ostringstream output;
  output << graph;

  std::cout << "# Use this snippet with your favorite DOT tool." << std::endl;
  std::cout << graph << std::endl;

  for (auto const &s : {"graph {\n",
                        "  0 [label=\"v0 (0)\"];\n",
                        "  1 [label=\"v1 (1)\"];\n",
                        "  2 [label=\"v2 (2)\"];\n",
                        "  3 [label=\"v3 (3)\"];\n"})
  {
    EXPECT_NE(output.str().find(s), std::string::npos);
  }

  // We don't really know the order in which the edges will be printed.
  // We also don't know the order in which the vertices on each edge will be
  // printed.
  std::vector<std::pair<std::string, std::string>> expectedEdges =
    {
      {"  0 -- 1;\n", "  1 -- 0;\n"},
      {"  1 -- 2;\n", "  2 -- 1;\n"},
      {"  0 -- 2;\n", "  2 -- 0;\n"}
    };
  for (auto const &edge : expectedEdges)
  {
    EXPECT_TRUE((output.str().find(std::get<0>(edge)) != std::string::npos) ||
                (output.str().find(std::get<1>(edge)) != std::string::npos));
  }
}
