/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

// Benchmarks for the tree-shaped algorithms (Ancestors, IsAncestor,
// LowestCommonAncestor, Subgraph, DescendantsSet) on a gz-sim-shaped
// entity tree. For stable numbers, pin to a single CPU using your
// platform's affinity tool (on Linux, e.g.,
// `taskset -c 1 ./bin/BENCHMARK_tree_algorithms`).

#include <benchmark/benchmark.h>

#include <cstdint>
#include <string>

#include "gz/math/graph/Graph.hh"
#include "gz/math/graph/GraphAlgorithms.hh"

using namespace gz;
using namespace math;
using namespace graph;

namespace {

using SimGraph = DirectedGraph<int, double>;

/// \brief Build a gz-sim-flavored entity tree: world -> models -> links ->
/// leaves. 1 + 50 + 500 + 2500 = 3051 entities, depth 3 from world.
SimGraph makeSimEntityTree()
{
  SimGraph g;
  VertexId nextId = 0;
  const VertexId world = nextId++;
  g.AddVertex("world", 0, world);
  for (std::size_t m = 0; m < 50; ++m)
  {
    const VertexId model = nextId++;
    g.AddVertex("model" + std::to_string(m), 0, model);
    g.AddEdge({world, model}, 0.0, 1.0);
    for (std::size_t l = 0; l < 10; ++l)
    {
      const VertexId link = nextId++;
      g.AddVertex("link", 0, link);
      g.AddEdge({model, link}, 0.0, 1.0);
      for (std::size_t k = 0; k < 5; ++k)
      {
        const VertexId leaf = nextId++;
        g.AddVertex("leaf", 0, leaf);
        g.AddEdge({link, leaf}, 0.0, 1.0);
      }
    }
  }
  return g;
}

// Vertex ids in makeSimEntityTree():
//   0   : world
//   1   : model0
//   3   : first leaf of model0's first link
//   64  : first leaf of model1's first link
//   3050: last entity in the tree (a deep leaf)
constexpr VertexId kWorld = 0;
constexpr VertexId kModel0 = 1;
constexpr VertexId kLeafM0 = 3;
constexpr VertexId kLeafM1 = 64;
constexpr VertexId kDeepLeaf = 3050;

}  // namespace

/////////////////////////////////////////////////
static void BM_AncestorsDeepLeaf(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    auto chain = Ancestors(g, kDeepLeaf);
    benchmark::DoNotOptimize(chain);
  }
}
BENCHMARK(BM_AncestorsDeepLeaf);

/////////////////////////////////////////////////
static void BM_IsAncestorWorldLeaf(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    bool r = IsAncestor(g, kWorld, kDeepLeaf);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_IsAncestorWorldLeaf);

/////////////////////////////////////////////////
static void BM_LowestCommonAncestorCrossModel(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    auto lca = LowestCommonAncestor(g, kLeafM0, kLeafM1);
    benchmark::DoNotOptimize(lca);
  }
}
BENCHMARK(BM_LowestCommonAncestorCrossModel);

/////////////////////////////////////////////////
static void BM_LowestCommonAncestorSameModel(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  // Both leaves under model0 but different links.
  for (auto _ : _state)
  {
    auto lca = LowestCommonAncestor(g, VertexId{3}, VertexId{9});
    benchmark::DoNotOptimize(lca);
  }
}
BENCHMARK(BM_LowestCommonAncestorSameModel);

/////////////////////////////////////////////////
static void BM_SubgraphModel0(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    auto sub = Subgraph(g, kModel0);
    benchmark::DoNotOptimize(sub);
  }
}
BENCHMARK(BM_SubgraphModel0);

/////////////////////////////////////////////////
static void BM_DescendantsSetWorld(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    auto set = DescendantsSet(g, kWorld);
    benchmark::DoNotOptimize(set);
  }
}
BENCHMARK(BM_DescendantsSetWorld);

/////////////////////////////////////////////////
static void BM_DescendantsSetModel0(benchmark::State &_state)
{
  auto g = makeSimEntityTree();
  for (auto _ : _state)
  {
    auto set = DescendantsSet(g, kModel0);
    benchmark::DoNotOptimize(set);
  }
}
BENCHMARK(BM_DescendantsSetModel0);

BENCHMARK_MAIN();
