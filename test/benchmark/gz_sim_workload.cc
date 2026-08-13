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

// Benchmarks that mirror gz-sim's real graph-API hot paths so the impact of
// gz-math/graph changes can be evaluated head-to-head against the workload
// downstream consumers actually run.
//
// For stable numbers, run pinned to a single CPU using your platform's
// affinity tool (on Linux, e.g., `taskset -c 1 ./bin/BENCHMARK_gz_sim_workload`).

#include <benchmark/benchmark.h>

#include <cstdint>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "gz/math/graph/Graph.hh"
#include "gz/math/graph/GraphAlgorithms.hh"

using namespace gz;
using namespace math;
using namespace graph;

namespace {

using SimGraph = DirectedGraph<int, double>;

/// \brief Build a gz-sim-flavored entity tree: world -> models -> links ->
/// leaves. Mirrors the shape of EntityComponentManager::Entities() in a
/// typical simulation.
/// \param[in] _numModels Number of model entities under the world.
/// \param[in] _linksPerModel Number of link entities under each model.
/// \param[in] _leavesPerLink Number of leaf entities under each link.
/// \return The constructed directed tree.
SimGraph makeSimEntityTree(std::size_t _numModels,
                           std::size_t _linksPerModel,
                           std::size_t _leavesPerLink)
{
  SimGraph g;
  VertexId nextId = 0;
  const VertexId world = nextId++;
  g.AddVertex("world", 0, world);
  for (std::size_t m = 0; m < _numModels; ++m)
  {
    const VertexId model = nextId++;
    g.AddVertex("model" + std::to_string(m), 0, model);
    g.AddEdge({world, model}, 0.0, 1.0);
    for (std::size_t l = 0; l < _linksPerModel; ++l)
    {
      const VertexId link = nextId++;
      g.AddVertex("link", 0, link);
      g.AddEdge({model, link}, 0.0, 1.0);
      for (std::size_t k = 0; k < _leavesPerLink; ++k)
      {
        const VertexId leaf = nextId++;
        g.AddVertex("leaf", 0, leaf);
        g.AddEdge({link, leaf}, 0.0, 1.0);
      }
    }
  }
  return g;
}

/// \brief Walk the parent chain from a leaf to the root, composing a
/// dummy double op at each step so the compiler cannot elide the work.
/// Models the per-tick "compose world transform" pattern that systems
/// run when iterating sensors / links.
/// \param[in] _g Directed entity tree (parent -> child).
/// \param[in] _leaf Vertex to start the walk from.
/// \return The accumulated dummy "pose" value.
double worldPoseStyleWalk(const SimGraph &_g, VertexId _leaf)
{
  double pose = 1.0;
  VertexId cur = _leaf;
  while (true)
  {
    auto parents = _g.AdjacentsTo(cur);
    if (parents.empty())
      break;
    cur = parents.begin()->first;
    pose = pose * 1.000001 + 0.5;
  }
  return pose;
}

/// \brief Walk the parent chain from a leaf to the root, concatenating
/// vertex names into a dotted scoped name. Models the pattern used by
/// downstream consumers to build "world.model.link" identifiers for
/// pub/sub topics, debug output, and SDF generation.
/// \param[in] _g Directed entity tree (parent -> child).
/// \param[in] _leaf Vertex to start the walk from.
/// \return The dotted scoped name "root.…leaf".
std::string scopedNameStyleWalk(const SimGraph &_g, VertexId _leaf)
{
  std::string out;
  VertexId cur = _leaf;
  while (true)
  {
    const auto &v = _g.VertexFromId(cur);
    out.insert(0, v.Name());
    auto parents = _g.AdjacentsTo(cur);
    if (parents.empty())
      break;
    out.insert(0, ".");
    cur = parents.begin()->first;
  }
  return out;
}

/// \brief Test a batch of vertex ids for membership: VertexFromId lookup
/// followed by a validity check. Mixing valid and invalid ids stresses
/// the underlying find().
/// \param[in] _g Source graph.
/// \param[in] _ids Vertex ids to test for membership.
/// \return Number of ids that resolved to a valid vertex.
std::size_t hasEntityBatch(const SimGraph &_g,
                           const std::vector<VertexId> &_ids)
{
  std::size_t hits = 0;
  for (auto id : _ids)
  {
    if (_g.VertexFromId(id).Id() != kNullId)
      ++hits;
  }
  return hits;
}

/// \brief BFS from a root and copy the results into an unordered_set.
/// Models the per-removal "find this subtree" lookup that runs whenever
/// an entity hierarchy is torn down.
/// \param[in] _g Source graph.
/// \param[in] _root Root vertex of the subtree.
/// \return Set of vertex ids reachable from _root.
std::unordered_set<VertexId> descendantsBfsCopy(
    const SimGraph &_g, VertexId _root)
{
  auto vec = BreadthFirstSort(_g, _root);
  return std::unordered_set<VertexId>(vec.begin(), vec.end());
}

/// \brief Scan every vertex and edge of a candidate graph and count the
/// ones absent from a live graph. Stresses Vertices()/Edges() iteration
/// plus VertexFromId and EdgeFromVertices lookup -- the same shape as a
/// scene-graph merge.
/// \param[in] _live The currently-live scene graph.
/// \param[in] _candidate Candidate graph to merge in.
/// \return Number of vertices+edges in _candidate that are absent from _live.
std::size_t sceneMergeStyle(const SimGraph &_live,
                            const SimGraph &_candidate)
{
  std::size_t newOnes = 0;
  for (auto const &kv : _candidate.Vertices())
  {
    if (!_live.VertexFromId(kv.first).Valid())
      ++newOnes;
  }
  for (auto const &kv : _candidate.Edges())
  {
    auto vs = kv.second.get().Vertices();
    if (!_live.EdgeFromVertices(vs.first, vs.second).Valid())
      ++newOnes;
  }
  return newOnes;
}

}  // namespace

/////////////////////////////////////////////////
static void BM_WorldPoseStyleWalk100Leaves(benchmark::State &_state)
{
  // 1 + 50 + 500 + 2500 = 3051 entities. Deepest leaf has depth 3 from world.
  auto g = makeSimEntityTree(50, 10, 5);
  std::mt19937 rng(0xCAFE);
  // Leaf ids only.
  std::uniform_int_distribution<VertexId> pick(551, 3050);
  std::vector<VertexId> leaves(100);
  for (auto &x : leaves)
    x = pick(rng);

  for (auto _ : _state)
  {
    double s = 0;
    for (auto v : leaves)
      s += worldPoseStyleWalk(g, v);
    benchmark::DoNotOptimize(s);
  }
}
BENCHMARK(BM_WorldPoseStyleWalk100Leaves);

/////////////////////////////////////////////////
static void BM_ScopedNameStyleWalk100Leaves(benchmark::State &_state)
{
  auto g = makeSimEntityTree(50, 10, 5);
  std::mt19937 rng(0xCAFE);
  std::uniform_int_distribution<VertexId> pick(551, 3050);
  std::vector<VertexId> leaves(100);
  for (auto &x : leaves)
    x = pick(rng);

  for (auto _ : _state)
  {
    std::size_t totalLen = 0;
    for (auto v : leaves)
      totalLen += scopedNameStyleWalk(g, v).size();
    benchmark::DoNotOptimize(totalLen);
  }
}
BENCHMARK(BM_ScopedNameStyleWalk100Leaves);

/////////////////////////////////////////////////
static void BM_HasEntityRandomBatch1000(benchmark::State &_state)
{
  auto g = makeSimEntityTree(50, 10, 5);
  std::mt19937 rng(0xCAFE);
  // Mix valid and invalid ids to stress the find().
  std::uniform_int_distribution<VertexId> pick(0, 5000);
  std::vector<VertexId> ids(1000);
  for (auto &x : ids)
    x = pick(rng);

  for (auto _ : _state)
  {
    auto r = hasEntityBatch(g, ids);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_HasEntityRandomBatch1000);

/////////////////////////////////////////////////
static void BM_DescendantsWorld(benchmark::State &_state)
{
  auto g = makeSimEntityTree(50, 10, 5);
  for (auto _ : _state)
  {
    auto r = descendantsBfsCopy(g, VertexId{0});
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_DescendantsWorld);

/////////////////////////////////////////////////
static void BM_DescendantsModel(benchmark::State &_state)
{
  auto g = makeSimEntityTree(50, 10, 5);
  for (auto _ : _state)
  {
    auto r = descendantsBfsCopy(g, VertexId{1});
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_DescendantsModel);

/////////////////////////////////////////////////
static void BM_SceneMergeAddEntities(benchmark::State &_state)
{
  auto live = makeSimEntityTree(50, 10, 5);
  // Slightly bigger candidate: 5 extra models so the merge has work to do.
  auto cand = makeSimEntityTree(55, 10, 5);
  for (auto _ : _state)
  {
    auto r = sceneMergeStyle(live, cand);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_SceneMergeAddEntities);

BENCHMARK_MAIN();
