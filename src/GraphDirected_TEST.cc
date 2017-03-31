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
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{0, 1, 0.0}, {1, 2, 0.0}}
  });

  // Verify the vertices.
  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  for (auto i = 0; i < 3; ++i)
  {
    auto v = graph.VertexById(i);
    ASSERT_TRUE(v != nullptr);
    EXPECT_EQ(v->Name(), std::to_string(i));
    EXPECT_EQ(v->Id(), i);
    EXPECT_EQ(v->Data(), i);
  }

  // Verify the edges.
  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 2u);
}

/////////////////////////////////////////////////
TEST(GraphTest, VertexById)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  EXPECT_EQ(v0->Name(), "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  auto v = graph.VertexById(v0->Id());
  ASSERT_TRUE(v != nullptr);
  EXPECT_EQ(v, v0);

  // Id not found.
  v = graph.VertexById(-1);
  ASSERT_EQ(v, nullptr);
}

/////////////////////////////////////////////////
TEST(GraphTest, vertices)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 3u);

  // Check that the pointers point to the same vertices.
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), v0), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), v1), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), v2), vertices.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, VerticesNames)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);
  auto v3 = graph.AddVertex(3, "vertex_2");
  ASSERT_TRUE(v3 != nullptr);

  auto vertices = graph.Vertices("vertex_2");
  EXPECT_EQ(vertices.size(), 2);
  // Check that the pointers point to the same vertices.
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), v2), vertices.end());
  EXPECT_NE(std::find(vertices.begin(), vertices.end(), v3), vertices.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, Edges)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Check that the pointers point to the same edges.
  EXPECT_NE(std::find(edges.begin(), edges.end(), e0), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), e1), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), e2), edges.end());
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
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Try to get the adjacents of an inexistent vertex.
  auto adjacents = graph.Adjacents(-1);
  EXPECT_TRUE(adjacents.empty());

  adjacents = graph.Adjacents(v0);
  EXPECT_EQ(adjacents.size(), 1u);
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v1), adjacents.end());

  adjacents = graph.Adjacents(0);
  EXPECT_EQ(adjacents.size(), 1u);
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v1), adjacents.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, Incidents)
{
  DirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  auto incidents = graph.Incidents(v0);
  EXPECT_EQ(incidents.size(), 1u);
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e2), incidents.end());

  incidents = graph.Incidents(0);
  EXPECT_EQ(incidents.size(), 1u);
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e2), incidents.end());
}

/////////////////////////////////////////////////
TEST(GraphTest, AddVertex)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create a vertex with Id.
  auto v3 = graph.AddVertex(5, "3");
  ASSERT_TRUE(v3 != nullptr);
  EXPECT_EQ(v3->Id(), 3);
  EXPECT_EQ(v3->Data(), 5);

  // Create a vertex with an already used Id.
  auto v4 = graph.AddVertex(0, "3", 3);
  ASSERT_TRUE(v4 == nullptr);

  auto vertices = graph.Vertices();
  EXPECT_EQ(vertices.size(), 4u);
}

/////////////////////////////////////////////////
TEST(GraphTest, AddEdge)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Check the edge content.
  EXPECT_EQ(e0->Data(), 2.0);
  EXPECT_EQ(e1->Data(), 3.0);
  EXPECT_EQ(e2->Data(), 4.0);

  // Check that the edges point to the right vertices.
  EXPECT_EQ(e0->Tail(), v0);
  EXPECT_EQ(e0->Head(), v1);

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect tail.
  auto edge = graph.AddEdge(nullptr, v1, 2.0);
  EXPECT_EQ(edge, nullptr);
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect head.
  edge = graph.AddEdge(v0, nullptr, 2.0);
  EXPECT_EQ(edge, nullptr);
  EXPECT_EQ(edges.size(), 3u);
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveEdge)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 3u);

  // Remove using nullptr shouldn't cause any effect.
  DirectedEdgePtr<int, double> edge;
  EXPECT_FALSE(graph.RemoveEdge(edge));
  EXPECT_EQ(graph.Edges().size(), 3u);

  EXPECT_EQ(graph.Incidents(v1).size(), 1);

  EXPECT_NE(e0->Head(), nullptr);
  EXPECT_NE(e0->Tail(), nullptr);

  EXPECT_TRUE(graph.RemoveEdge(e0));
  EXPECT_EQ(graph.Edges().size(), 2u);
  // After disconnecting e0, it shoudln't be possible to reach the vertices.
  EXPECT_EQ(e0->Head(), nullptr);
  EXPECT_EQ(e0->Tail(), nullptr);

  EXPECT_EQ(graph.Incidents(v1).size(), 0);

  EXPECT_TRUE(graph.RemoveEdge(e1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  // Try to remove an edge that doesn't exist.
  EXPECT_FALSE(graph.RemoveEdge(e1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_TRUE(graph.RemoveEdge(e2));
  EXPECT_EQ(graph.Edges().size(), 0u);
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVertex)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 3u);

  // Remove using nullptr shouldn't cause any effect.
  VertexPtr<int> vertex;
  EXPECT_FALSE(graph.RemoveVertex(vertex));
  EXPECT_EQ(graph.Vertices().size(), 3u);

  // Try to remove a vertex that doesn't belong to the graph.
  auto v = std::make_shared<Vertex<int>>("new_vertex", 99, 10);
  EXPECT_FALSE(graph.RemoveVertex(v));
  EXPECT_EQ(graph.Vertices().size(), 3u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 1);

  EXPECT_TRUE(graph.RemoveVertex(2));
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 0);

  EXPECT_TRUE(graph.RemoveVertex(v1));
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertex(v0));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVerticesWithName)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);
  auto v3 = graph.AddVertex(3, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v3), (v3-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v3, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  auto e3 = graph.AddEdge(v3, v0, 5.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 4u);

  // Try to remove a node with a name that doesn't exist.
  EXPECT_FALSE(graph.RemoveVertices("wrong_name"));
  EXPECT_EQ(graph.Vertices().size(), 4u);
  EXPECT_EQ(graph.Adjacents(v1).size(), 1);

  EXPECT_TRUE(graph.RemoveVertices("vertex_2"));
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 0);

  EXPECT_TRUE(graph.RemoveVertices("vertex_1"));
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertices("vertex_0"));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, StreamInsertion)
{
  DirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);
  auto v3 = graph.AddVertex(3, "vertex_3");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto e0 = graph.AddEdge(v0, v1, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  auto e1 = graph.AddEdge(v1, v2, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  auto e2 = graph.AddEdge(v2, v0, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 3u);

  std::ostringstream output;
  output << graph;

  std::cout << "# Use this snippet with your favorite DOT tool." << std::endl;
  std::cout << graph << std::endl;

  for (auto const &s : {"digraph {\n",
                        "  0 [label=\"vertex_0 (0)\"];\n",
                        "  1 [label=\"vertex_1 (1)\"];\n",
                        "  2 [label=\"vertex_2 (2)\"];\n",
                        "  3 [label=\"vertex_3 (3)\"];\n",
                        "  0 -> 1;\n",
                        "  1 -> 2;\n",
                        "  2 -> 0;\n"})
  {
    EXPECT_NE(output.str().find(s), std::string::npos);
  }
}
