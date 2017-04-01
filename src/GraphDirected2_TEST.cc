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

#include "ignition/math/GraphDirected.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(GraphTest, UniformInitialization)
{
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 0.0}, {1, 2, 0.0}}
  });

  // Verify the vertices.
  auto vertices = graph._Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  for (auto const &id : vertices)
  {
    auto v = graph._VertexById(id);
    EXPECT_EQ(v.Name(), std::to_string(id));
    EXPECT_EQ(v.Id(), id);
    EXPECT_EQ(v.Data(), id);
  }

  // Verify the edges.
  auto edges = graph._Edges();
  EXPECT_EQ(edges.size(), 2u);
}

/////////////////////////////////////////////////
TEST(GraphTest, VertexById)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph._AddVertex(0, "0");
  EXPECT_EQ(v0.Name(), "0");
  auto v1 = graph._AddVertex(1, "1");
  auto v2 = graph._AddVertex(2, "2");

  auto v = graph._VertexById(v0.Id());
  EXPECT_EQ(v.Id(), v0.Id());

  // Id not found.
  v = graph._VertexById(-500);
  EXPECT_EQ(v.Id(), kNullId);
}

/////////////////////////////////////////////////
TEST(GraphTest, vertices)
{
  DirectedGraph<int, double> graph(
  {
    {{0, "0"}, {1, "1"}, {2, "2"}},
    {{0, 1, 0.0}, {1, 2, 0.0}}
  });

  auto vertices = graph._Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  // Check that the vertices Ids start from 0.
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), 0), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), 1), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), 2), vertices.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, VerticesNames)
{
  // Create a few vertices with two of them sharing the same name.
  DirectedGraph<int, double> graph(
  {
    {{0, "vertex_0"}, {1, "vertex_1"}, {2, "common"}, {3, "common"}},
    {}
  });

  auto vertices = graph._Vertices("common");
  EXPECT_EQ(vertices.size(), 2);
  // Check the Ids.
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), 2), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), 3), vertices.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, Edges)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  auto edges = graph._Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Check the Ids.
  EXPECT_NE(std::find(edges.begin(), edges.end(), 0), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), 1), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), 2), edges.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, Empty)
{
  DirectedGraph<int, double> graph;

  EXPECT_TRUE(graph.Empty());

  // Create a vertex.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);

  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, Adjacents)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  // Try to get the adjacents of an inexistent vertex.
  auto adjacents = graph._Adjacents(kNullId);
  EXPECT_TRUE(adjacents.empty());

  adjacents = graph._Adjacents(0);
  EXPECT_EQ(adjacents.size(), 1u);
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), 1), adjacents.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, Incidents)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  auto incidents = graph._Incidents(0);
  EXPECT_EQ(incidents.size(), 1u);
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), 2), incidents.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, AddVertex)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph._AddVertex(0, "0");
  EXPECT_TRUE(v0.Id() != kNullId);
  auto v1 = graph._AddVertex(1, "1");
  EXPECT_TRUE(v1.Id() != kNullId);
  auto v2 = graph._AddVertex(2, "2");
  EXPECT_TRUE(v2.Id() != kNullId);

  // Create a vertex with Id.
  auto v3 = graph._AddVertex(5, "3", 3);
  EXPECT_EQ(v3.Id(), 3);
  EXPECT_EQ(v3.Data(), 5);
  EXPECT_EQ(v3.Name(), "3");

  // Create a vertex with an already used Id.
  auto v4 = graph._AddVertex(0, "3", 3);
  ASSERT_TRUE(v4.Id() == kNullId);

  auto vertices = graph._Vertices();
  EXPECT_EQ(vertices.size(), 4u);
}

/////////////////////////////////////////////////
TEST(GraphTest, AddEdge)
{
  // Create a graph with three vertices.
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {}
  });

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(0, 1, 2.0);
  auto e1 = graph.AddEdge(1, 2, 3.0);
  auto e2 = graph.AddEdge(2, 0, 4.0);

  // Check the edge content.
  EXPECT_EQ(e0.Data(), 2.0);
  EXPECT_EQ(e1.Data(), 3.0);
  EXPECT_EQ(e2.Data(), 4.0);

  // Check that the edges point to the right vertices.
  EXPECT_EQ(e0.Tail(), 0);
  EXPECT_EQ(e0.Head(), 1);

  auto edges = graph._Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect tail.
  auto edge = graph.AddEdge(kNullId, 1, 2.0);
  EXPECT_EQ(edge.Id(), kNullId);
  EXPECT_EQ(graph._Edges().size(), 3u);

  // Try to add an edge with an incorrect head.
  edge = graph.AddEdge(0, kNullId, 2.0);
  EXPECT_EQ(edge.Id(), kNullId);
  EXPECT_EQ(graph._Edges().size(), 3u);
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveEdge)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  // Remove a nonexistent edge shouldn't cause any effect.
  EXPECT_FALSE(graph._RemoveEdge(kNullId));
  EXPECT_EQ(graph._Edges().size(), 3u);
  EXPECT_EQ(graph._Incidents(1).size(), 1);

  // Remove the edge (v0-->v1)
  EXPECT_TRUE(graph._RemoveEdge(0));
  EXPECT_EQ(graph._Edges().size(), 2u);
  EXPECT_EQ(graph._Incidents(1).size(), 0);

  // Remove the edge (v1-->v2)
  EXPECT_TRUE(graph._RemoveEdge(1));
  EXPECT_EQ(graph._Edges().size(), 1u);

  // Try to remove an edge that doesn't exist anymore.
  EXPECT_FALSE(graph._RemoveEdge(1));
  EXPECT_EQ(graph._Edges().size(), 1u);

  // Remove the edge (v2-->v0)
  EXPECT_TRUE(graph._RemoveEdge(2));
  EXPECT_EQ(graph._Edges().size(), 0u);
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVertex)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "0", 0}, {1, "1", 1}, {2, "2", 2}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  // Remove a nonexistent vertex shouldn't cause any effect.
  EXPECT_FALSE(graph._RemoveVertex(kNullId));
  EXPECT_EQ(graph._Vertices().size(), 3u);
  EXPECT_EQ(graph._Adjacents(1).size(), 1u);

  // Remove vertex #2.
  EXPECT_TRUE(graph._RemoveVertex(2));
  EXPECT_EQ(graph._Vertices().size(), 2u);
  EXPECT_EQ(graph._Edges().size(), 1u);
  EXPECT_EQ(graph._Adjacents(1).size(), 0u);

  // Remove vertex #1.
  EXPECT_TRUE(graph._RemoveVertex(1));
  EXPECT_EQ(graph._Vertices().size(), 1u);
  EXPECT_TRUE(graph._Edges().empty());

  // Remove vertex #0.
  EXPECT_TRUE(graph._RemoveVertex(0));
  EXPECT_TRUE(graph._Vertices().empty());

  EXPECT_TRUE(graph._Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVerticesWithName)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v3), (v3-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{0, "v0", 0}, {1, "v1", 1}, {2, "common", 2}, {3, "common", 3}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 3, 4.0}, {3, 0, 5.0}}
  });

  // Try to remove a node with a name that doesn't exist.
  EXPECT_FALSE(graph._RemoveVertices("wrong_name"));
  EXPECT_EQ(graph._Vertices().size(), 4u);
  EXPECT_EQ(graph._Adjacents(1).size(), 1);

  // Remove two vertices at the same time.
  EXPECT_TRUE(graph._RemoveVertices("common"));
  EXPECT_EQ(graph._Vertices().size(), 2u);
  EXPECT_EQ(graph._Edges().size(), 1u);

  EXPECT_EQ(graph._Adjacents(1).size(), 0);

  EXPECT_TRUE(graph._RemoveVertices("v1"));
  EXPECT_EQ(graph._Vertices().size(), 1u);
  EXPECT_TRUE(graph._Edges().empty());

  EXPECT_TRUE(graph._RemoveVertices("v0"));
  EXPECT_TRUE(graph._Vertices().empty());

  EXPECT_TRUE(graph._Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, StreamInsertion)
{
  // Create a graph with 4 vertices and edges [(v0-->v1), (v1-->v2), (v2-->v3)]
  DirectedGraph<int, double> graph(
  {
    {{0, "v0", 0}, {1, "v1", 1}, {2, "v2", 2}, {3, "v3", 3}},
    {{0, 1, 2.0}, {1, 2, 3.0}, {2, 0, 4.0}}
  });

  std::ostringstream output;
  output << graph;

  std::cout << "# Use this snippet with your favorite DOT tool." << std::endl;
  std::cout << graph << std::endl;

  for (auto const &s : {"digraph {\n",
                        "  0 [label=\"v0 (0)\"];\n",
                        "  1 [label=\"v1 (1)\"];\n",
                        "  2 [label=\"v2 (2)\"];\n",
                        "  3 [label=\"v3 (3)\"];\n",
                        "  0 -> 1;\n",
                        "  1 -> 2;\n",
                        "  2 -> 0;\n"})
  {
    EXPECT_NE(output.str().find(s), std::string::npos);
  }
}
