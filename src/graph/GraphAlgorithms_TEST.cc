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
#include <unordered_set>

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
TYPED_TEST_SUITE(GraphTestFixture, GraphTypes, );

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

/////////////////////////////////////////////////
// Tree algorithm: walk the parent chain of a leaf back to the root.
// Verifies both the chain contents and the reachedRoot flag.
TEST(GraphAlgorithmsTree, Ancestors_LinearChain)
{
  // 0 - 1 - 2 - 3   (parent pointers; AddEdge follows parent->child).
  DirectedGraph<int, double> g;
  for (int i = 0; i < 4; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0);
  g.AddEdge({1, 2}, 0.0);
  g.AddEdge({2, 3}, 0.0);

  // Deep leaf: walks to root cleanly.
  auto r = Ancestors(g, 3u);
  EXPECT_EQ(r.first, (std::vector<VertexId>{2, 1, 0}));
  EXPECT_TRUE(r.second);

  // Mid-chain: walks to root cleanly.
  r = Ancestors(g, 1u);
  EXPECT_EQ(r.first, (std::vector<VertexId>{0}));
  EXPECT_TRUE(r.second);

  // Root vertex itself: empty chain, but reachedRoot is true (already at root).
  r = Ancestors(g, 0u);
  EXPECT_TRUE(r.first.empty());
  EXPECT_TRUE(r.second);
}

/////////////////////////////////////////////////
// Tree algorithm: invalid input vertex returns an empty chain with
// reachedRoot=false (the walk could not start).
TEST(GraphAlgorithmsTree, Ancestors_InvalidVertex)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  auto r = Ancestors(g, 99u);
  EXPECT_TRUE(r.first.empty());
  EXPECT_FALSE(r.second);
}

/////////////////////////////////////////////////
// Tree algorithm: Ancestors() must terminate even if the graph contains
// a cycle. The chain returned stops at the first revisit and reachedRoot
// is false to flag the cycle abort.
TEST(GraphAlgorithmsTree, Ancestors_ToleratesCycle)
{
  DirectedGraph<int, double> g;
  for (int i = 0; i < 3; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0);
  g.AddEdge({1, 2}, 0.0);
  g.AddEdge({2, 0}, 0.0);  // cycle back to 0

  // Walking up from 2: parent is 1, then 0; the cycle would re-visit 2,
  // which is the seen sentinel -- stop without infinite loop.
  auto r = Ancestors(g, 2u);
  EXPECT_EQ(r.first.size(), 2u);
  EXPECT_EQ(r.first[0], 1u);
  EXPECT_EQ(r.first[1], 0u);
  EXPECT_FALSE(r.second);
}

/////////////////////////////////////////////////
// Tree algorithm: IsAncestor walks the descendant up its parent chain.
TEST(GraphAlgorithmsTree, IsAncestor)
{
  // world(0) -> modelA(1) -> linkA(3)
  // world(0) -> modelB(2) -> linkB(4)
  DirectedGraph<int, double> g;
  for (int i = 0; i < 5; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0); g.AddEdge({1, 3}, 0.0);
  g.AddEdge({0, 2}, 0.0); g.AddEdge({2, 4}, 0.0);

  EXPECT_TRUE(IsAncestor(g, 0u, 3u));   // world is ancestor of linkA
  EXPECT_TRUE(IsAncestor(g, 1u, 3u));   // modelA is ancestor of linkA
  EXPECT_FALSE(IsAncestor(g, 1u, 4u));  // modelA is NOT ancestor of linkB
  EXPECT_FALSE(IsAncestor(g, 3u, 1u));  // child is not ancestor of parent
  EXPECT_FALSE(IsAncestor(g, 0u, 0u));  // strict ancestor (a != d)
  EXPECT_FALSE(IsAncestor(g, 99u, 3u));  // bogus inputs
}

/////////////////////////////////////////////////
// Tree algorithm: IsAncestor must reject an invalid descendant id.
TEST(GraphAlgorithmsTree, IsAncestor_InvalidDescendant)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 1}, 0.0);

  EXPECT_FALSE(IsAncestor(g, 0u, 99u));
}

/////////////////////////////////////////////////
// Tree algorithm: IsAncestor must terminate when the descendant chain
// contains a cycle that does not include the candidate ancestor.
TEST(GraphAlgorithmsTree, IsAncestor_ToleratesCycle)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddVertex("v2", 2, 2);
  g.AddVertex("v3", 3, 3);  // isolated -- the candidate ancestor.
  // Cycle 0 -> 1 -> 2 -> 0, none of which connect to vertex 3.
  g.AddEdge({0, 1}, 0.0);
  g.AddEdge({1, 2}, 0.0);
  g.AddEdge({2, 0}, 0.0);

  EXPECT_FALSE(IsAncestor(g, 3u, 2u));
}

/////////////////////////////////////////////////
// Tree algorithm: lowest common ancestor of two vertices in a forest.
TEST(GraphAlgorithmsTree, LowestCommonAncestor)
{
  // world -> modelA -> linkA1
  //                 -> linkA2
  //       -> modelB -> linkB1
  DirectedGraph<int, double> g;
  for (int i = 0; i < 6; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  // 0 = world; 1 = modelA; 2 = modelB
  // 3 = linkA1; 4 = linkA2; 5 = linkB1
  g.AddEdge({0, 1}, 0.0); g.AddEdge({0, 2}, 0.0);
  g.AddEdge({1, 3}, 0.0); g.AddEdge({1, 4}, 0.0);
  g.AddEdge({2, 5}, 0.0);

  // Two siblings under modelA -> LCA = modelA.
  EXPECT_EQ(LowestCommonAncestor(g, 3u, 4u), 1u);
  // linkA1 vs linkB1 -> LCA = world.
  EXPECT_EQ(LowestCommonAncestor(g, 3u, 5u), 0u);
  // Ancestor case: LCA(world, linkA1) = world.
  EXPECT_EQ(LowestCommonAncestor(g, 0u, 3u), 0u);
  // LCA(x, x) = x.
  EXPECT_EQ(LowestCommonAncestor(g, 4u, 4u), 4u);
}

/////////////////////////////////////////////////
// Tree algorithm: vertices in disjoint trees have no LCA.
TEST(GraphAlgorithmsTree, LowestCommonAncestor_DisjointTrees)
{
  DirectedGraph<int, double> g;
  for (int i = 0; i < 4; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  // Two disjoint chains: 0->1, 2->3.
  g.AddEdge({0, 1}, 0.0); g.AddEdge({2, 3}, 0.0);

  EXPECT_EQ(LowestCommonAncestor(g, 1u, 3u), kNullId);
}

/////////////////////////////////////////////////
// Tree algorithm: LCA returns kNullId when either input is invalid.
TEST(GraphAlgorithmsTree, LowestCommonAncestor_InvalidInputs)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 1}, 0.0);

  EXPECT_EQ(LowestCommonAncestor(g, 99u, 1u), kNullId);
  EXPECT_EQ(LowestCommonAncestor(g, 0u, 99u), kNullId);
}

/////////////////////////////////////////////////
// Tree algorithm: LCA hits the early-return shortcut when `_b` is itself
// an ancestor of `_a`.
TEST(GraphAlgorithmsTree, LowestCommonAncestor_BIsAncestorOfA)
{
  // 0 -> 1 -> 2 -> 3
  DirectedGraph<int, double> g;
  for (int i = 0; i < 4; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0);
  g.AddEdge({1, 2}, 0.0);
  g.AddEdge({2, 3}, 0.0);

  // _a = 3 (deep leaf), _b = 1 (its grandparent) -- _b is on _a's chain.
  EXPECT_EQ(LowestCommonAncestor(g, 3u, 1u), 1u);
}

/////////////////////////////////////////////////
// Tree algorithm: Subgraph extracts the reachable region rooted at a vertex.
TEST(GraphAlgorithmsTree, Subgraph_ExtractsModel)
{
  // world -> modelA -> linkA1, linkA2
  //       -> modelB -> linkB1
  DirectedGraph<int, double> g;
  for (int i = 0; i < 6; ++i)
    g.AddVertex("v" + std::to_string(i), i * 10, i);
  g.AddEdge({0, 1}, 0.0); g.AddEdge({0, 2}, 0.0);
  g.AddEdge({1, 3}, 0.0); g.AddEdge({1, 4}, 0.0);
  g.AddEdge({2, 5}, 0.0);

  // Extract the subtree rooted at modelA.
  auto sub = Subgraph(g, 1u);
  EXPECT_EQ(sub.Vertices().size(), 3u);  // modelA + linkA1 + linkA2
  EXPECT_EQ(sub.Edges().size(),    2u);  // modelA->linkA1, modelA->linkA2
  EXPECT_TRUE(sub.VertexFromId(1).Valid());
  EXPECT_TRUE(sub.VertexFromId(3).Valid());
  EXPECT_TRUE(sub.VertexFromId(4).Valid());
  EXPECT_FALSE(sub.VertexFromId(0).Valid());  // world not in subtree
  EXPECT_FALSE(sub.VertexFromId(5).Valid());  // linkB1 not in subtree

  // Data preserved.
  EXPECT_EQ(sub.VertexFromId(3).Data(), 30);
}

/////////////////////////////////////////////////
// Tree algorithm: Subgraph returns an empty graph for an invalid root.
TEST(GraphAlgorithmsTree, Subgraph_InvalidRoot)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 1}, 0.0);

  auto sub = Subgraph(g, 99u);
  EXPECT_TRUE(sub.Vertices().empty());
  EXPECT_TRUE(sub.Edges().empty());
}

/////////////////////////////////////////////////
// Tree algorithm: Subgraph preserves cycles within the reachable region.
// (This is the reason the function is named Subgraph and not Subtree:
// the result is not necessarily a tree.)
TEST(GraphAlgorithmsTree, Subgraph_PreservesCycles)
{
  // Reachable region forms a cycle: 0 -> 1 -> 2 -> 0, plus a chain
  // hanging off vertex 1. All four vertices reachable from 0.
  DirectedGraph<int, double> g;
  for (int i = 0; i < 4; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0);
  g.AddEdge({1, 2}, 0.0);
  g.AddEdge({2, 0}, 0.0);  // back-edge closing the cycle
  g.AddEdge({1, 3}, 0.0);

  auto sub = Subgraph(g, 0u);
  EXPECT_EQ(sub.Vertices().size(), 4u);
  // All four edges (including the back-edge that forms the cycle) preserved.
  EXPECT_EQ(sub.Edges().size(), 4u);
  EXPECT_TRUE(sub.EdgeFromVertices(2u, 0u).Valid());
}

/////////////////////////////////////////////////
// Tree algorithm: DescendantsSet returns the same vertex set as a
// BreadthFirstSort copy into a set.
TEST(GraphAlgorithmsTree, DescendantsSet_AgreesWithBFS)
{
  DirectedGraph<int, double> g;
  for (int i = 0; i < 5; ++i)
    g.AddVertex("v" + std::to_string(i), i, i);
  g.AddEdge({0, 1}, 0.0); g.AddEdge({0, 2}, 0.0);
  g.AddEdge({1, 3}, 0.0); g.AddEdge({2, 4}, 0.0);

  auto bfs = BreadthFirstSort(g, 0u);
  std::unordered_set<VertexId> bfsSet(bfs.begin(), bfs.end());
  auto desc = DescendantsSet(g, 0u);
  EXPECT_EQ(desc, bfsSet);
}

/////////////////////////////////////////////////
// Tree algorithm: DescendantsSet returns an empty set for an invalid id.
TEST(GraphAlgorithmsTree, DescendantsSet_InvalidVertex)
{
  DirectedGraph<int, double> g;
  g.AddVertex("v0", 0, 0);
  g.AddVertex("v1", 1, 1);
  g.AddEdge({0, 1}, 0.0);

  EXPECT_TRUE(DescendantsSet(g, 99u).empty());
}
