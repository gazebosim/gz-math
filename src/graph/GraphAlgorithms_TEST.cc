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
#include <string>

#include "gz/math/graph/Graph.hh"
#include "gz/math/graph/GraphAlgorithms.hh"

using namespace gz;
using namespace math;
using namespace graph;

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
TYPED_TEST(GraphTestFixture, BreadthFirstSort)
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

  auto res = BreadthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 1, 2, 4, 3, 5, 6};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(GraphTest, DepthFirstSortDirected)
{
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0}, {{1, 3}, 2.0},
     {{1, 5}, 3.0}, {{2, 6}, 4.0}, {{5, 4}, 2.0}}
  });

  auto res = DepthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 4, 2, 6, 1, 5, 3};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, DepthFirstSortUndirected)
{
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0}, {{1, 3}, 2.0},
     {{1, 5}, 3.0}, {{2, 6}, 4.0}, {{5, 4}, 2.0}}
  });

  auto res = DepthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 4, 5, 1, 3, 2, 6};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, DijkstraUndirected)
{
  ///              (6)                 |
  ///           0-------1              |
  ///           |      /|\             |
  ///           |     / | \(5)         |
  ///           | (2)/  |  \           |
  ///           |   /   |   2          |
  ///        (1)|  / (2)|  /           |
  ///           | /     | /(5)         |
  ///           |/      |/             |
  ///           3-------4              |
  ///              (1)                 |
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"3", 3, 3}, {"4", 4, 4}},
    // Edges.
    {{{0, 1}, 2.0, 6.0}, {{0, 3}, 3.0, 1.0},
     {{1, 2}, 4.0, 5.0}, {{1, 3}, 4.0, 2.0}, {{1, 4}, 4.0, 2.0},
     {{2, 4}, 2.0, 5.0},
     {{3, 4}, 2.0, 1.0}}
  });

  // Inexistent source vertex.
  auto res = Dijkstra(graph, 99);
  EXPECT_TRUE(res.empty());

  // Inexistent destination vertex.
  res = Dijkstra(graph, 0, 99);
  EXPECT_TRUE(res.empty());

  // Calculate all shortest paths from 0.
  res = Dijkstra(graph, 0);

  ASSERT_NE(res.end(), res.find(0));
  EXPECT_DOUBLE_EQ(0, res.at(0).first);
  EXPECT_EQ(0u, res.at(0).second);
  ASSERT_NE(res.end(), res.find(1));
  EXPECT_DOUBLE_EQ(3, res.at(1).first);
  EXPECT_EQ(3u, res.at(1).second);
  ASSERT_NE(res.end(), res.find(2));
  EXPECT_DOUBLE_EQ(7, res.at(2).first);
  EXPECT_EQ(4u, res.at(2).second);
  ASSERT_NE(res.end(), res.find(3));
  EXPECT_DOUBLE_EQ(1, res.at(3).first);
  EXPECT_EQ(0u, res.at(3).second);
  ASSERT_NE(res.end(), res.find(4));
  EXPECT_DOUBLE_EQ(2, res.at(4).first);
  EXPECT_EQ(3u, res.at(4).second);

  // Calculate the shortest path between 0 and 1.
  res = Dijkstra(graph, 0, 1);

  ASSERT_NE(res.end(), res.find(1));
  EXPECT_DOUBLE_EQ(3, res.at(1).first);
  EXPECT_EQ(3u, res.at(1).second);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, DijkstraDirected)
{
  ///              (6)                  |
  ///           0------>1               |
  ///           |      /|\              |
  ///           |     / | \(5)          |
  ///           | (2)/  |  ┘            |
  ///           |   /   |   2           |
  ///        (1)|  / (2)|  /            |
  ///           | /     | /(5)          |
  ///           VL      VL              |
  ///           3------>4               |
  ///              (1)                  |
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"3", 3, 3}, {"4", 4, 4}},
    // Edges.
    {{{0, 1}, 2.0, 6.0}, {{0, 3}, 3.0, 1.0},
     {{1, 2}, 4.0, 5.0}, {{1, 3}, 4.0, 2.0}, {{1, 4}, 4.0, 2.0},
     {{2, 4}, 2.0, 5.0},
     {{3, 4}, 2.0, 1.0}}
  });

  // Inexistent source vertex.
  auto res = Dijkstra(graph, 99);
  EXPECT_TRUE(res.empty());

  // Inexistent destination vertex.
  res = Dijkstra(graph, 0, 99);
  EXPECT_TRUE(res.empty());

  // Calculate all shortest paths from 0.
  res = Dijkstra(graph, 0);

  ASSERT_NE(res.end(), res.find(0));
  EXPECT_DOUBLE_EQ(0, res.at(0).first);
  EXPECT_EQ(0u, res.at(0).second);
  ASSERT_NE(res.end(), res.find(1));
  EXPECT_DOUBLE_EQ(6, res.at(1).first);
  EXPECT_EQ(0u, res.at(1).second);
  ASSERT_NE(res.end(), res.find(2));
  EXPECT_DOUBLE_EQ(11, res.at(2).first);
  EXPECT_EQ(1u, res.at(2).second);
  ASSERT_NE(res.end(), res.find(3));
  EXPECT_DOUBLE_EQ(1, res.at(3).first);
  EXPECT_EQ(0u, res.at(3).second);
  ASSERT_NE(res.end(), res.find(4));
  EXPECT_DOUBLE_EQ(2, res.at(4).first);
  EXPECT_EQ(3u, res.at(4).second);

  // Calculate the shortest path between 0 and 1.
  res = Dijkstra(graph, 0, 1);

  ASSERT_NE(res.end(), res.find(1));
  EXPECT_DOUBLE_EQ(6, res.at(1).first);
  EXPECT_EQ(0u, res.at(1).second);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, ConnectedComponents)
{
  // Connected components of an empty graph.
  UndirectedGraph<int, double> emptyGraph;
  auto components = ConnectedComponents(emptyGraph);
  EXPECT_TRUE(components.empty());

  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4}},
    // Edges.
    {{{0, 2}, 2.0, 6.0},
     {{1, 4}, 4.0, 5.0}}
  });

  // Connected components of a graph with three components.
  components = ConnectedComponents(graph);
  ASSERT_EQ(3u, components.size());

  // Component #0.
  auto component = components.at(0);
  auto vertices = component.Vertices();
  EXPECT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(2));
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 0:
      {
        EXPECT_EQ("A", vertex.Name());
        EXPECT_EQ(0, vertex.Data());
        break;
      }
      case 2:
      {
        EXPECT_EQ("C", vertex.Name());
        EXPECT_EQ(2, vertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }

  // Component #1.
  component = components.at(1);
  vertices = component.Vertices();
  EXPECT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(1));
  EXPECT_NE(vertices.end(), vertices.find(4));
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 1:
      {
        EXPECT_EQ("B", vertex.Name());
        EXPECT_EQ(1, vertex.Data());
        break;
      }
      case 4:
      {
        EXPECT_EQ("E", vertex.Name());
        EXPECT_EQ(4, vertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }

  // Component #2.
  component = components.at(2);
  vertices = component.Vertices();
  EXPECT_EQ(1u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(3));
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 3:
      {
        EXPECT_EQ("D", vertex.Name());
        EXPECT_EQ(3, vertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, ToUndirectedGraph)
{
  ///              (6)                  |
  ///           0------>1               |
  ///           |      /|\              |
  ///           |     / | \(5)          |
  ///           | (2)/  |  ┘            |
  ///           |   /   |   2           |
  ///        (1)|  / (2)|  /            |
  ///           | /     | /(5)          |
  ///           VL      VL              |
  ///           3------>4               |
  ///              (1)                  |
  DirectedGraph<int, double> directed(
  {
    // Vertices.
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"3", 3, 3}, {"4", 4, 4}},
    // Edges.
    {{{0, 1}, 2.0, 6.0}, {{0, 3}, 3.0, 1.0},
     {{1, 2}, 4.0, 5.0}, {{1, 3}, 4.0, 2.0}, {{1, 4}, 4.0, 2.0},
     {{2, 4}, 2.0, 5.0},
     {{3, 4}, 2.0, 1.0}}
  });

  // Convert to undirected graph.
  auto undirected = ToUndirectedGraph(directed);
  EXPECT_EQ(5u, undirected.Vertices().size());
  EXPECT_EQ(7u, undirected.Edges().size());
  EXPECT_EQ(directed.Vertices().size(), undirected.Vertices().size());
  EXPECT_EQ(directed.Edges().size(), undirected.Edges().size());

  // Compare vertices.
  for (auto const &dvPair : directed.Vertices())
  {
    const VertexId dvId = dvPair.first;
    auto const &uv = undirected.VertexFromId(dvId);
    EXPECT_TRUE(uv.Valid());
    EXPECT_TRUE(dvPair.second.get().Valid());
    EXPECT_EQ(uv.Id(), dvId);
    EXPECT_EQ(uv.Name(), dvPair.second.get().Name());
    EXPECT_EQ(uv.Data(), dvPair.second.get().Data());
  }

  // Compare edges.
  for (auto const &dePair : directed.Edges())
  {
    const EdgeId deId = dePair.first;
    auto const &ue = undirected.EdgeFromId(deId);
    EXPECT_TRUE(ue.Valid());
    EXPECT_TRUE(dePair.second.get().Valid());
    EXPECT_EQ(ue.Id(), deId);
    EXPECT_DOUBLE_EQ(ue.Data(), dePair.second.get().Data());
    EXPECT_DOUBLE_EQ(ue.Weight(), dePair.second.get().Weight());
  }

  // std::cerr << directed << std::endl;
  // std::cerr << undirected << std::endl;
}

/////////////////////////////////////////////////
// Pin Dijkstra correctness on a graph that triggers many relaxation
// updates: stale priority-queue entries left behind by relaxation must
// not produce wrong distances.
TEST(GraphAlgorithmsBugfix, DijkstraStaleEntrySkipPreservesCorrectness)
{
  // Diamond with a long way around to force relaxations.
  // 0 - 1 - 2 - 3
  // |           |
  //  ----- 4 ----
  UndirectedGraph<int, double> g;
  for (int i = 0; i < 5; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0, 1.0);
  g.AddEdge({1, 2}, 0.0, 1.0);
  g.AddEdge({2, 3}, 0.0, 1.0);
  g.AddEdge({0, 4}, 0.0, 10.0);
  g.AddEdge({4, 3}, 0.0, 10.0);

  auto dist = Dijkstra(g, 0);
  EXPECT_DOUBLE_EQ(0.0,  dist[0].first);
  EXPECT_DOUBLE_EQ(1.0,  dist[1].first);
  EXPECT_DOUBLE_EQ(2.0,  dist[2].first);
  EXPECT_DOUBLE_EQ(3.0,  dist[3].first);
  EXPECT_DOUBLE_EQ(10.0, dist[4].first);
}

/////////////////////////////////////////////////
// Coverage: BFS on a single-vertex graph returns just that vertex.
TEST(GraphAlgorithmsCoverage, BFS_SingleVertex)
{
  UndirectedGraph<int, double> g;
  g.AddVertex("only", 0, 0);
  auto bfs = BreadthFirstSort(g, 0);
  ASSERT_EQ(bfs.size(), 1u);
  EXPECT_EQ(bfs[0], 0u);

  auto dfs = DepthFirstSort(g, 0);
  ASSERT_EQ(dfs.size(), 1u);
  EXPECT_EQ(dfs[0], 0u);
}

/////////////////////////////////////////////////
// Coverage: BFS / DFS visit a self-looped vertex exactly once.
TEST(GraphAlgorithmsCoverage, BFS_SelfLoopVisitsOnce)
{
  UndirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 0}, 0.0, 1.0);  // self-loop
  g.AddEdge({0, 1}, 0.0, 1.0);

  auto bfs = BreadthFirstSort(g, 0);
  EXPECT_EQ(bfs.size(), 2u);
  EXPECT_EQ(std::count(bfs.begin(), bfs.end(), 0u), 1);

  auto dfs = DepthFirstSort(g, 0);
  EXPECT_EQ(dfs.size(), 2u);
  EXPECT_EQ(std::count(dfs.begin(), dfs.end(), 0u), 1);
}

/////////////////////////////////////////////////
// Coverage: ConnectedComponents on an empty graph returns no components.
TEST(GraphAlgorithmsCoverage, ConnectedComponents_EmptyGraph)
{
  UndirectedGraph<int, double> g;
  EXPECT_TRUE(ConnectedComponents(g).empty());
}

/////////////////////////////////////////////////
// Coverage: a graph with vertices but no edges has one component per
// vertex (each is its own singleton component).
TEST(GraphAlgorithmsCoverage, ConnectedComponents_DisconnectedSingletons)
{
  UndirectedGraph<int, double> g;
  for (int i = 0; i < 4; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);

  auto components = ConnectedComponents(g);
  EXPECT_EQ(components.size(), 4u);
  for (auto const &c : components)
    EXPECT_EQ(c.Vertices().size(), 1u);
}

/////////////////////////////////////////////////
// Coverage: Dijkstra picks the cheapest of multiple parallel edges
// between the same pair of vertices (multigraph behavior).
TEST(GraphAlgorithmsCoverage, Dijkstra_MultipleEdgesBetweenSamePair)
{
  UndirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 1}, 0.0, 5.0);
  g.AddEdge({0, 1}, 0.0, 1.0);

  auto dist = Dijkstra(g, 0);
  EXPECT_DOUBLE_EQ(dist[1].first, 1.0);
}

/////////////////////////////////////////////////
// Coverage: BFS and DFS return an empty result when the source vertex is
// not part of the graph. Pre-rewrite they would walk the bogus id once and
// return a single-element vector containing it.
TEST(GraphAlgorithmsCoverage, BFS_DFS_MissingSourceReturnsEmpty)
{
  UndirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);

  EXPECT_TRUE(BreadthFirstSort(g, 99).empty());
  EXPECT_TRUE(DepthFirstSort(g, 99).empty());
}

/////////////////////////////////////////////////
// Coverage: BFS and DFS on a graph with no vertices return an empty result.
TEST(GraphAlgorithmsCoverage, BFS_DFS_EmptyGraphReturnsEmpty)
{
  UndirectedGraph<int, double> g;

  EXPECT_TRUE(BreadthFirstSort(g, 0).empty());
  EXPECT_TRUE(DepthFirstSort(g, 0).empty());
}

/////////////////////////////////////////////////
// Coverage: BFS and DFS work correctly when vertex ids are sparse and
// scattered across a large numeric range.
TEST(GraphAlgorithmsCoverage, BFS_DFS_SparseHighIds)
{
  UndirectedGraph<int, double> g;
  g.AddVertex("a", 0, 1000000);
  g.AddVertex("b", 0, 2000000);
  g.AddVertex("c", 0, 3000000);
  g.AddEdge({1000000, 2000000}, 0.0, 1.0);
  g.AddEdge({2000000, 3000000}, 0.0, 1.0);

  auto bfs = BreadthFirstSort(g, 1000000);
  EXPECT_EQ(bfs.size(), 3u);

  auto dfs = DepthFirstSort(g, 1000000);
  EXPECT_EQ(dfs.size(), 3u);
}
