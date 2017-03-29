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

  // Verify the vertexes.
  auto vertexes = graph.Vertexes();
  EXPECT_EQ(vertexes.size(), 3u);

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

  // Create some vertexes.
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
TEST(UndirectedGraphTest, Vertexes)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  auto vertexes = graph.Vertexes();
  EXPECT_EQ(vertexes.size(), 3u);

  // Check that the pointers point to the same vertexes.
  EXPECT_NE(std::find(vertexes.begin(), vertexes.end(), v0), vertexes.end());
  EXPECT_NE(std::find(vertexes.begin(), vertexes.end(), v1), vertexes.end());
  EXPECT_NE(std::find(vertexes.begin(), vertexes.end(), v2), vertexes.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, VertexesNames)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);
  auto v3 = graph.AddVertex(3, "vertex_2");
  ASSERT_TRUE(v3 != nullptr);

  auto vertexes = graph.Vertexes("vertex_2");
  EXPECT_EQ(vertexes.size(), 2);
  // Check that the pointers point to the same vertexes.
  EXPECT_NE(std::find(vertexes.begin(), vertexes.end(), v2), vertexes.end());
  EXPECT_NE(std::find(vertexes.begin(), vertexes.end(), v3), vertexes.end());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, Edges)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
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

  // Create some vertexes.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
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

  // Create some vertexes.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
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

  // Create some vertexes without Id.
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

  auto vertexes = graph.Vertexes();
  EXPECT_EQ(vertexes.size(), 4u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, AddEdge)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Try to add an edge with just one vertex.
  vertexes = {v0};
  auto e3 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  // Check the edge content.
  EXPECT_EQ(e0->Data(), 2.0);
  EXPECT_EQ(e1->Data(), 3.0);
  EXPECT_EQ(e2->Data(), 4.0);

  // Check that the edges point to the right vertexes.
  EXPECT_NE(e0->Vertexes().find(v0), e0->Vertexes().end());
  EXPECT_NE(e0->Vertexes().find(v1), e0->Vertexes().end());

  auto edges = graph.Edges();
  EXPECT_EQ(edges.size(), 3u);

  // Try to add an edge with an incorrect vertex.
  VertexPtr_S<int> wrongVertexes = {nullptr, v1};
  auto edge = graph.AddEdge(wrongVertexes, 2.0);
  EXPECT_EQ(edge, nullptr);
  EXPECT_EQ(edges.size(), 3u);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveEdge)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  EXPECT_EQ(graph.Edges().size(), 3u);

  // Remove an edge with vertexes that don't exist.

  // Remove using nullptr shouldn't cause any effect.
  UndirectedEdgePtr<int, double> edge;
  EXPECT_FALSE(graph.RemoveEdge(edge));
  EXPECT_EQ(graph.Edges().size(), 3u);

  EXPECT_EQ(graph.Incidents(v1).size(), 2u);

  vertexes = e0->Vertexes();
  EXPECT_EQ(vertexes.find(nullptr), vertexes.end());

  EXPECT_TRUE(graph.RemoveEdge(e0));
  EXPECT_EQ(graph.Edges().size(), 2u);

  // After disconnecting e0, it shoudln't be possible to reach the vertexes.
  vertexes = e0->Vertexes();
  EXPECT_NE(vertexes.find(nullptr), vertexes.end());

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

  // Create some vertexes without Id.
  auto v0 = graph.AddVertex(0, "0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  EXPECT_EQ(graph.Edges().size(), 3u);

  // Remove using nullptr shouldn't cause any effect.
  VertexPtr<int> vertex;
  EXPECT_FALSE(graph.RemoveVertex(vertex));
  EXPECT_EQ(graph.Vertexes().size(), 3u);

  // Try to remove a vertex that doesn't belong to the graph.
  auto v = std::make_shared<Vertex<int>>(10, "new_vertex", 99);
  EXPECT_FALSE(graph.RemoveVertex(v));
  EXPECT_EQ(graph.Vertexes().size(), 3u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 2u);

  EXPECT_TRUE(graph.RemoveVertex(2));
  EXPECT_EQ(graph.Vertexes().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 1u);

  EXPECT_TRUE(graph.RemoveVertex(v1));
  EXPECT_EQ(graph.Vertexes().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertex(v0));
  EXPECT_TRUE(graph.Vertexes().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, RemoveVertexesWithName)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes without Id.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);
  auto v3 = graph.AddVertex(3, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v3), (v3-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v3};
  auto e2 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);
  vertexes = {v3, v0};
  auto e3 = graph.AddEdge(vertexes, 5.0);
  ASSERT_TRUE(e3 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 4u);

  // Try to remove a node with a name that doesn't exist.
  EXPECT_FALSE(graph.RemoveVertexes("wrong_name"));
  EXPECT_EQ(graph.Vertexes().size(), 4u);
  EXPECT_EQ(graph.Adjacents(v1).size(), 2u);

  EXPECT_TRUE(graph.RemoveVertexes("vertex_2"));
  EXPECT_EQ(graph.Vertexes().size(), 2u);
  EXPECT_EQ(graph.Edges().size(), 1u);

  EXPECT_EQ(graph.Adjacents(v1).size(), 1u);

  EXPECT_TRUE(graph.RemoveVertexes("vertex_1"));
  EXPECT_EQ(graph.Vertexes().size(), 1u);
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_TRUE(graph.RemoveVertexes("vertex_0"));
  EXPECT_TRUE(graph.Vertexes().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, StreamInsertion)
{
  UndirectedGraph<int, double> graph;

  // Create some vertexes without Id.
  auto v0 = graph.AddVertex(0, "vertex_0");
  ASSERT_TRUE(v0 != nullptr);
  auto v1 = graph.AddVertex(1, "vertex_1");
  ASSERT_TRUE(v1 != nullptr);
  auto v2 = graph.AddVertex(2, "vertex_2");
  ASSERT_TRUE(v2 != nullptr);

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  VertexPtr_S<int> vertexes = {v0, v1};
  auto e0 = graph.AddEdge(vertexes, 2.0);
  ASSERT_TRUE(e0 != nullptr);
  vertexes = {v1, v2};
  auto e1 = graph.AddEdge(vertexes, 3.0);
  ASSERT_TRUE(e1 != nullptr);
  vertexes = {v2, v0};
  auto e2 = graph.AddEdge(vertexes, 4.0);
  ASSERT_TRUE(e2 != nullptr);

  EXPECT_EQ(graph.Edges().size(), 3u);

  std::ostringstream output;
  output << graph;

  for (auto const &s : {"Vertexes\n",
                        "  [0][vertex_0]\n",
                        "  [1][vertex_1]\n",
                        "  [2][vertex_2]\n",
                        "Edges\n"})
  {
    EXPECT_NE(output.str().find(s), std::string::npos);
  }

  // We don't really know the order in which the edges will be printed.
  // We also don't know the order in which the vertexes on each edge will be
  // printed.
  std::vector<std::pair<std::string, std::string>> expectedEdges =
    {
      {"  [0]--[1]\n", "  [1]--[0]\n"},
      {"  [1]--[2]\n", "  [2]--[1]\n"},
      {"  [0]--[2]\n", "  [0]--[2]\n"}
    };
  for (auto const &edge : expectedEdges)
  {
    EXPECT_TRUE((output.str().find(std::get<0>(edge)) != std::string::npos) ||
                (output.str().find(std::get<1>(edge)) != std::string::npos));
  }
}
