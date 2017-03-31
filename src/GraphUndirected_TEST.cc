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
#include <string>
#include <utility>
#include <vector>

#include "ignition/math/GraphUndirected.hh"

using namespace ignition;
using namespace math;


/////////////////////////////////////////////////
TEST(UndirectedGraphTest, UniformInitialization)
{
  UndirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
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
TEST(UndirectedGraphTest, VertexById)
{
  UndirectedGraph<int, double> graph;

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
TEST(UndirectedGraphTest, Vertices)
{
  UndirectedGraph<int, double> graph;

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
TEST(UndirectedGraphTest, VerticesNames)
{
  UndirectedGraph<int, double> graph;

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
TEST(UndirectedGraphTest, Edges)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Check that the pointers point to the same edges.
  EXPECT_NE(std::find(edges.begin(), edges.end(), e0), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), e1), edges.end());
  EXPECT_NE(std::find(edges.begin(), edges.end(), e2), edges.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Empty)
{
  UndirectedGraph<int, double> graph;

  EXPECT_TRUE(graph.Empty());

  // Create a vertex.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);

  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Adjacents)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  auto adjacents = graph.Adjacents(v0);
  EXPECT_EQ(adjacents.size(), 2u);
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v1), adjacents.end());
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v2), adjacents.end());

  adjacents = graph.Adjacents(0);
  EXPECT_EQ(adjacents.size(), 2u);
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v1), adjacents.end());
  EXPECT_NE(std::find(adjacents.begin(), adjacents.end(), v2), adjacents.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Incidents)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  auto incidents = graph.Incidents(v0);
  EXPECT_EQ(incidents.size(), 2u);
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e2), incidents.end());
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e0), incidents.end());

  incidents = graph.Incidents(0);
  EXPECT_EQ(incidents.size(), 2u);
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e2), incidents.end());
  EXPECT_NE(std::find(incidents.begin(), incidents.end(), e0), incidents.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, AddVertex)
{
  UndirectedGraph<int, double> graph;

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
TEST(UndirectedGraphTest, AddEdge)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Try to add an edge with just one vertex.
  vertices = {v0};
  auto e3 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Check the edge content.
  EXPECT_EQ(e0->Data(), 2.0);
  EXPECT_EQ(e1->Data(), 3.0);
  EXPECT_EQ(e2->Data(), 4.0);

  // Check that the edges point to the right vertices.
  EXPECT_NE(e0->Vertices().find(v0), e0->Vertices().end());
  EXPECT_NE(e0->Vertices().find(v1), e0->Vertices().end());

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect vertex.
  VertexPtr_S<int> wrongVertices = {nullptr, v1};
  auto edge = graph.AddEdge(wrongVertices, 2.0);
  EXPECT_EQ(edge, nullptr);
  EXPECT_EQ(edges.size(), 3u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveEdge)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  EXPECT_EQ(graph.Edges().size(), 3u);

  // Remove an edge with vertices that don't exist.

  // Remove using nullptr shouldn't cause any effect.
  UndirectedEdgePtr<int, double> edge;
  EXPECT_FALSE(graph.RemoveEdge(edge));
  EXPECT_EQ(graph.Edges().size(), 3u);

  EXPECT_EQ(graph.Incidents(v1).size(), 2u);

  vertices = e0->Vertices();
  EXPECT_EQ(vertices.find(nullptr), vertices.end());

  EXPECT_TRUE(graph.RemoveEdge(e0));
  EXPECT_EQ(graph.Edges().size(), 2u);

  // After disconnecting e0, it shoudln't be possible to reach the vertices.
  vertices = e0->Vertices();
  EXPECT_NE(vertices.find(nullptr), vertices.end());

  EXPECT_EQ(graph.Incidents(v1).size(), 1u);

  EXPECT_TRUE(graph.RemoveEdge(e1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  // Try to remove an edge that doesn't exist.
  EXPECT_FALSE(graph.RemoveEdge(e1));
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_TRUE(graph.RemoveEdge(e2));
  EXPECT_EQ(graph.Edges().size(), 0u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveVertex)
{
  UndirectedGraph<int, double> graph;

  // Create some vertices without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
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

  EXPECT_EQ(graph.Adjacents(v1).size(), 2u);

  EXPECT_TRUE(graph.RemoveVertex(2));
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 1u);

  EXPECT_TRUE(graph.RemoveVertex(v1));
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertex(v0));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveVerticesWithName)
{
  UndirectedGraph<int, double> graph;

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
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v3};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  vertices = {v3, v0};
  auto e3 = graph.AddEdge(vertices, 5.0);
  ASSERT_TRUE(e3 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 4u);

  // Try to remove a node with a name that doesn't exist.
  EXPECT_FALSE(graph.RemoveVertices("wrong_name"));
  EXPECT_EQ(graph.Vertices().size(), 4u);
  EXPECT_EQ(graph.Adjacents(v1).size(), 2u);

  EXPECT_TRUE(graph.RemoveVertices("vertex_2"));
  EXPECT_EQ(graph.Vertices().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 1u);

  EXPECT_TRUE(graph.RemoveVertices("vertex_1"));
  EXPECT_EQ(graph.Vertices().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertices("vertex_0"));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, StreamInsertion)
{
  UndirectedGraph<int, double> graph;

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
  VertexPtr_S<int> vertices = {v0, v1};
  auto e0 = graph.AddEdge(vertices, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertices = {v1, v2};
  auto e1 = graph.AddEdge(vertices, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertices = {v2, v0};
  auto e2 = graph.AddEdge(vertices, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 3u);

  std::ostringstream output;
  output << graph;

  std::cout << "# Use this snippet with your favorite DOT tool." << std::endl;
  std::cout << graph << std::endl;

  for (auto const &s : {"graph {\n",
                        "  0 [label=\"vertex_0 (0)\"];\n",
                        "  1 [label=\"vertex_1 (1)\"];\n",
                        "  2 [label=\"vertex_2 (2)\"];\n",
                        "  3 [label=\"vertex_3 (3)\"];\n"})
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
