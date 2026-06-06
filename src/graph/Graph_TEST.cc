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
#include <string>

#include "gz/math/graph/Graph.hh"

using namespace gz;
using namespace math;
using namespace graph;

// Define a test fixture class template.
template <class T>
class GraphTestFixture : public testing::Test
{
};

// Simulate a class with the maximum allowed number of vertices used.
template <typename V, typename E>
class MockVerticesFullUndirectedGraph : public UndirectedGraph<V, E>
{
  // Default constructor.
  public: MockVerticesFullUndirectedGraph()
  {
    this->nextVertexId = MAX_UI64;
  }
};

// Simulate a class with the maximum allowed number of edges used.
template <typename V, typename E>
class MockEdgesFullUndirectedGraph : public UndirectedGraph<V, E>
{
  // Default constructor.
  public: MockEdgesFullUndirectedGraph()
  {
    this->nextEdgeId = MAX_UI64;
  }
};

// The list of graphs we want to test.
using GraphTypes = ::testing::Types<DirectedGraph<int, double>,
                                    UndirectedGraph<int, double>>;
TYPED_TEST_SUITE(GraphTestFixture, GraphTypes, );

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, UniformInitialization)
{
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
      // Create edges.
      {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(3u, vertices.size());

    for (int i = 0; i < 3; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(2u, edges.size());
  }
  {
    TypeParam graph(
    {
      // Create vertices with automatic Id selection.
      {{"0", 0}, {"1", 1}, {"2", 2}},
      // Create edges.
      {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(3u, vertices.size());

    for (int i = 0; i < 3; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(2u, edges.size());
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, BadUniformInitializationMock)
{
  // There's no space for more vertices (mocked).
  {
    MockVerticesFullUndirectedGraph<int, double> graph;
    graph.AddVertex("0", 0);
    auto vertices = graph.Vertices();
    EXPECT_EQ(0u, vertices.size());
  }

  // There's no space for more edges (mocked).
  {
    MockEdgesFullUndirectedGraph<int, double> graph;
    graph.AddVertex("0", 0);
    graph.AddVertex("1", 1);
    graph.AddEdge({0, 1}, 1.0);
    auto edges = graph.Edges();
    EXPECT_EQ(0u, edges.size());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, BadUniformInitialization)
{
  // Bad graph initialization: repeated vertex Id.
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 0}, {"1", 1, 1}},
      // Create edges.
      {{{0, 1}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(2u, vertices.size());

    for (int i = 0; i < 2; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }
  }
  // Bad graph initialization: edges referencing an inexistent vertex.
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
      // Create edges.
      {{{0, 3}, 0.0}}
    });

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(0u, edges.size());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, VertexFromId)
{
  // Mutable version of VertexFromId().
  {
    TypeParam graph;

    // Create some vertices.
    auto &v0 = graph.AddVertex("0", 0, 0);
    EXPECT_EQ("0", v0.Name());
    graph.AddVertex("1", 1, 1);
    graph.AddVertex("2", 2, 2);

    auto v = graph.VertexFromId(v0.Id());
    EXPECT_EQ(v0.Id(), v.Id());

    // Id not found.
    v = graph.VertexFromId(500);
    EXPECT_EQ(kNullId, v.Id());
  }

  // Non-mutable version of VertexFromId().
  {
    const TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {}
    });

    auto v = graph.VertexFromId(0);
    EXPECT_EQ(0u, v.Id());

    // Id not found.
    v = graph.VertexFromId(500);
    EXPECT_EQ(kNullId, v.Id());
  }
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
  EXPECT_EQ(3u, vertices.size());

  // Check that the vertex Ids start from 0.
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));
  EXPECT_NE(vertices.end(), vertices.find(2));

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 0:
      {
        EXPECT_EQ("0", vertex.Name());
        EXPECT_EQ(10, vertex.Data());
        break;
      }
      case 1:
      {
        EXPECT_EQ("1", vertex.Name());
        EXPECT_EQ(20, vertex.Data());
        break;
      }
      case 2:
      {
        EXPECT_EQ("2", vertex.Name());
        EXPECT_EQ(30, vertex.Data());
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
  EXPECT_EQ(2u, vertices.size());

  // Check the Ids.
  EXPECT_NE(vertices.end(), vertices.find(2));
  EXPECT_NE(vertices.end(), vertices.find(3));

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 2:
      {
        EXPECT_EQ("common", vertex.Name());
        EXPECT_EQ(2, vertex.Data());
        break;
      }
      case 3:
      {
        EXPECT_EQ("common", vertex.Name());
        EXPECT_EQ(3, vertex.Data());
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
  auto &v0 = graph.AddVertex("0", 0);
  ASSERT_TRUE(v0.Valid());
  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, AddVertex)
{
  TypeParam graph;

  // Create some vertices without Id.
  auto &v0 = graph.AddVertex("0", 0);
  EXPECT_TRUE(v0.Id() != kNullId);
  auto &v1 = graph.AddVertex("1", 1);
  EXPECT_TRUE(v1.Id() != kNullId);
  auto &v2 = graph.AddVertex("2", 2);
  EXPECT_TRUE(v2.Id() != kNullId);

  // Create a vertex with Id.
  auto &v3 = graph.AddVertex("3", 5, 3);
  EXPECT_EQ(3u, v3.Id());
  EXPECT_EQ(5, v3.Data());
  EXPECT_EQ("3", v3.Name());

  // Create a vertex with an already used Id.
  auto &v4 = graph.AddVertex("3", 0, 3);
  ASSERT_TRUE(v4.Id() == kNullId);

  auto vertices = graph.Vertices();
  EXPECT_EQ(4u, vertices.size());

  // Change data in v3 and verify that is propagated into the graph.
  v3.Data() = 10;
  auto &vertex = graph.VertexFromId(v3.Id());
  EXPECT_EQ(10, vertex.Data());

  // Try to change data in v4 and verify that is not propagated into the graph.
  v4.Data() = 20;
  for (auto const &vertexPair : vertices)
  {
    auto &v = vertexPair.second.get();
    EXPECT_NE(20, v.Data());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgeFromVertices)
{
  TypeParam graph(
      {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3}}
      });

  const auto &edge = graph.EdgeFromVertices(0, 1);
  EXPECT_NE(kNullId, edge.Id());
  EXPECT_DOUBLE_EQ(2.0, edge.Data());

  {
    const auto &noEdge = graph.EdgeFromVertices(0, 2);
    EXPECT_EQ(kNullId, noEdge.Id());
  }

  {
    const auto &noEdge = graph.EdgeFromVertices(4, 5);
    EXPECT_EQ(kNullId, noEdge.Id());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgeFromId)
{
  // Mutable version of EdgeFromId().
  {
    TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3.0}}
    });

    auto e = graph.EdgeFromId(1);
    EXPECT_EQ(1u, e.Id());

    // Id not found.
    e = graph.EdgeFromId(500);
    EXPECT_EQ(kNullId, e.Id());
  }

  // Non-mutable version of EdgeFromId().
  {
    const TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3.0}}
    });

    auto e = graph.EdgeFromId(1);
    EXPECT_EQ(1u, e.Id());

    // Id not found.
    e = graph.EdgeFromId(500);
    EXPECT_EQ(kNullId, e.Id());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgelessInDegree)
{
  TypeParam graph;

  // add a bunch of vertices but no edges
  const int vertexCount = 10000;
  for (int i = 0; i < vertexCount; ++i)
  {
    auto &v = graph.AddVertex(std::to_string(i), i);
    EXPECT_TRUE(v.Valid());
  }

  for (auto const &idVertex : graph.Vertices())
  {
    EXPECT_EQ(0u, graph.InDegree(idVertex.first));
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgelessOutDegree)
{
  TypeParam graph;

  // add a bunch of vertices but no edges
  const int vertexCount = 10000;
  for (int i = 0; i < vertexCount; ++i)
  {
    auto &v = graph.AddVertex(std::to_string(i), i);
    EXPECT_TRUE(v.Valid());
  }

  for (auto const &idVertex : graph.Vertices())
  {
    EXPECT_EQ(0u, graph.OutDegree(idVertex.first));
  }
}

/////////////////////////////////////////////////
// LinkEdge is the public entry point that AddEdge delegates to. It
// accepts a caller-built edge object (id supplied by the caller),
// validates both endpoints exist, and inserts the edge into the
// graph (updating adjacency and the cached Edges() view). Exercised
// here for both directed and undirected graphs; AddEdge tests only
// cover the indirect path.
template <typename GraphT, typename EdgeT>
void LinkEdgeCoverage()
{
  GraphT graph;
  graph.AddVertex("0", 0, 0);
  graph.AddVertex("1", 1, 1);
  // Vertex id 2 is intentionally absent so we can exercise the
  // missing-endpoint branches.

  // Happy path: both endpoints exist.
  auto &linked = graph.LinkEdge(EdgeT({0, 1}, 7.5, 1.0, 100));
  EXPECT_TRUE(linked.Valid());
  EXPECT_EQ(100u, linked.Id());
  EXPECT_DOUBLE_EQ(7.5, linked.Data());

  // The cached Edges() view must reflect the newly linked edge.
  const auto &edges = graph.Edges();
  ASSERT_EQ(1u, edges.size());
  EXPECT_EQ(linked.Id(), edges.begin()->first);

  // EdgeFromId and adjacency must agree with the cache.
  EXPECT_EQ(linked.Id(), graph.EdgeFromId(linked.Id()).Id());
  EXPECT_EQ(1u, graph.IncidentsFrom(0).size());
  EXPECT_EQ(1u, graph.IncidentsTo(1).size());

  // Both endpoints missing: returns NullEdge, cache unchanged.
  auto &missingBoth = graph.LinkEdge(EdgeT({42, 43}, 0.0, 1.0, 101));
  EXPECT_FALSE(missingBoth.Valid());
  EXPECT_EQ(1u, graph.Edges().size());

  // Source endpoint missing: returns NullEdge, cache unchanged.
  auto &missingSrc = graph.LinkEdge(EdgeT({42, 1}, 0.0, 1.0, 102));
  EXPECT_FALSE(missingSrc.Valid());
  EXPECT_EQ(1u, graph.Edges().size());

  // Destination endpoint missing: returns NullEdge, cache unchanged.
  auto &missingDst = graph.LinkEdge(EdgeT({0, 42}, 0.0, 1.0, 103));
  EXPECT_FALSE(missingDst.Valid());
  EXPECT_EQ(1u, graph.Edges().size());
}

/////////////////////////////////////////////////
TEST(GraphTest, LinkEdgeDirected)
{
  LinkEdgeCoverage<DirectedGraph<int, double>, DirectedEdge<double>>();
}

/////////////////////////////////////////////////
TEST(GraphTest, LinkEdgeUndirected)
{
  LinkEdgeCoverage<UndirectedGraph<int, double>, UndirectedEdge<double>>();
}

/////////////////////////////////////////////////
// The cached Vertices() and Edges() views must stay consistent with
// the graph after removals. RemoveEdge must drop the edge from the
// Edges() cache; RemoveVertex must drop the vertex from the
// Vertices() cache and cascade to remove its incident edges from the
// Edges() cache; RemoveVertices(name) must do the same in bulk.
TYPED_TEST(GraphTestFixture, RemoveKeepsCachedAccessorsConsistent)
{
  // 0 -- 1 -- 2, plus a third vertex "1" sharing a name with vertex 1.
  TypeParam graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"1", 3, 3}},
    {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
  });

  // Sanity check the initial cached state.
  ASSERT_EQ(4u, graph.Vertices().size());
  ASSERT_EQ(2u, graph.Edges().size());

  const auto edge01Id = graph.EdgeFromVertices(0, 1).Id();
  const auto edge12Id = graph.EdgeFromVertices(1, 2).Id();
  ASSERT_NE(kNullId, edge01Id);
  ASSERT_NE(kNullId, edge12Id);

  // Removing an edge must update the cached Edges() view, leaving the
  // Vertices() cache untouched.
  EXPECT_TRUE(graph.RemoveEdge(edge01Id));
  {
    const auto &edges = graph.Edges();
    EXPECT_EQ(1u, edges.size());
    EXPECT_EQ(edges.end(), edges.find(edge01Id));
    EXPECT_NE(edges.end(), edges.find(edge12Id));
  }
  EXPECT_EQ(4u, graph.Vertices().size());

  // Removing a vertex must update the cached Vertices() view and
  // cascade to drop its incident edges from the Edges() cache.
  EXPECT_TRUE(graph.RemoveVertex(1));
  {
    const auto &vertices = graph.Vertices();
    EXPECT_EQ(3u, vertices.size());
    EXPECT_EQ(vertices.end(), vertices.find(1));
    EXPECT_NE(vertices.end(), vertices.find(0));
    EXPECT_NE(vertices.end(), vertices.find(2));
    EXPECT_NE(vertices.end(), vertices.find(3));
  }
  // The incident edge (1 -- 2) must be gone from the Edges() cache.
  EXPECT_TRUE(graph.Edges().empty());

  // RemoveVertices(name) must also keep the Vertices() cache in sync.
  // Only vertex 3 still carries the name "1" at this point.
  EXPECT_EQ(1u, graph.RemoveVertices("1"));
  {
    const auto &vertices = graph.Vertices();
    EXPECT_EQ(2u, vertices.size());
    EXPECT_EQ(vertices.end(), vertices.find(3));
  }
}

/////////////////////////////////////////////////
// Regression: renaming a graph-owned vertex via Vertex::SetName and
// then removing it via Graph::RemoveVertices(name) must find the
// vertex under its new name. Pre-fix, RemoveVertices read from an
// internal name->id index that SetName did not update, so the
// rename made the vertex un-removable by name (silent 0-return).
TEST(GraphTest, RemoveVerticesAfterSetNameWorks)
{
  UndirectedGraph<int, double> graph;
  graph.AddVertex("alice", 0, 0);
  graph.AddVertex("bob",   1, 1);

  // Rename through the mutable reference handed back by VertexFromId.
  graph.VertexFromId(0).SetName("alice2");

  // The vertex must be reachable under the new name.
  EXPECT_EQ(1u, graph.RemoveVertices("alice2"));
  EXPECT_FALSE(graph.VertexFromId(0).Valid());
  // Removing under the old name is a no-op now.
  EXPECT_EQ(0u, graph.RemoveVertices("alice"));
  // Bob is untouched.
  EXPECT_TRUE(graph.VertexFromId(1).Valid());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, CopyConstructor)
{
  TypeParam graph;
  graph.AddVertex("0", 0, 0);
  graph.AddVertex("1", 1, 1);
  graph.AddEdge({0, 1}, 2.0);

  // Copy the graph
  TypeParam graphCopy = graph;

  // Verify vertices in copy
  auto vertices = graphCopy.Vertices();
  ASSERT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));

  // Verify edges in copy
  auto edges = graphCopy.Edges();
  ASSERT_EQ(1u, edges.size());

  // Modify the original graph: change vertex data and remove it
  graph.VertexFromId(0).Data() = 99;
  graph.RemoveVertex(0);

  // The copy must remain unaffected
  auto verticesCopy = graphCopy.Vertices();
  ASSERT_EQ(2u, verticesCopy.size());
  ASSERT_NE(verticesCopy.end(), verticesCopy.find(0));
  EXPECT_EQ(0, verticesCopy.at(0).get().Data());
  EXPECT_EQ("0", verticesCopy.at(0).get().Name());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, CopyAssignment)
{
  TypeParam graph;
  graph.AddVertex("0", 0, 0);
  graph.AddVertex("1", 1, 1);
  graph.AddEdge({0, 1}, 2.0);

  TypeParam graphCopy;
  graphCopy = graph;

  // Verify vertices in copy
  auto vertices = graphCopy.Vertices();
  ASSERT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));

  // Verify edges in copy
  auto edges = graphCopy.Edges();
  ASSERT_EQ(1u, edges.size());

  // Modify the original graph
  graph.VertexFromId(0).Data() = 99;
  graph.RemoveVertex(0);

  // The copy must remain unaffected
  auto verticesCopy = graphCopy.Vertices();
  ASSERT_EQ(2u, verticesCopy.size());
  ASSERT_NE(verticesCopy.end(), verticesCopy.find(0));
  EXPECT_EQ(0, verticesCopy.at(0).get().Data());
  EXPECT_EQ("0", verticesCopy.at(0).get().Name());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, MoveConstructor)
{
  TypeParam graph;
  graph.AddVertex("0", 0, 0);
  graph.AddVertex("1", 1, 1);
  graph.AddEdge({0, 1}, 2.0);

  // Move the graph
  TypeParam graphMoved = std::move(graph);

  // Verify vertices in moved graph
  auto vertices = graphMoved.Vertices();
  ASSERT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));

  // Verify edges in moved graph
  auto edges = graphMoved.Edges();
  ASSERT_EQ(1u, edges.size());

  // Original graph must be empty
  EXPECT_TRUE(graph.Empty());
  EXPECT_EQ(0u, graph.Vertices().size());
  EXPECT_EQ(0u, graph.Edges().size());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, MoveAssignment)
{
  TypeParam graph;
  graph.AddVertex("0", 0, 0);
  graph.AddVertex("1", 1, 1);
  graph.AddEdge({0, 1}, 2.0);

  TypeParam graphMoved;
  graphMoved = std::move(graph);

  // Verify vertices in moved graph
  auto vertices = graphMoved.Vertices();
  ASSERT_EQ(2u, vertices.size());
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));

  // Verify edges in moved graph
  auto edges = graphMoved.Edges();
  ASSERT_EQ(1u, edges.size());

  // Original graph must be empty
  EXPECT_TRUE(graph.Empty());
  EXPECT_EQ(0u, graph.Vertices().size());
  EXPECT_EQ(0u, graph.Edges().size());
}
