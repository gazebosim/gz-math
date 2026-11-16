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

// Microbenchmarks for the gz-math graph module. Built only when
// google-benchmark is available (see test/benchmark/CMakeLists.txt). For
// stable numbers, run pinned to a single CPU using your platform's affinity
// tool (on Linux, e.g., `taskset -c 1 ./bin/BENCHMARK_graph`).

#include <benchmark/benchmark.h>

#include <cstdint>
#include <random>
#include <string>

#include "gz/math/graph/Graph.hh"
#include "gz/math/graph/GraphAlgorithms.hh"

using namespace gz;
using namespace math;
using namespace graph;

namespace {

/// \brief Build a deterministic sparse random graph for traversal/path
/// benchmarks. Uses a fixed seed so successive runs are bit-identical.
/// \param[in] _numVertices Number of vertices to create.
/// \param[in] _avgDegree Target average degree (controls density).
/// \param[in] _seed RNG seed for edge selection.
/// \return The constructed graph.
template <typename EdgeT>
Graph<int, double, EdgeT> makeRandomGraph(std::size_t _numVertices,
                                          double _avgDegree,
                                          std::uint32_t _seed = 0xC0FFEE)
{
  Graph<int, double, EdgeT> g;
  for (std::size_t i = 0; i < _numVertices; ++i)
  {
    g.AddVertex("v" + std::to_string(i), static_cast<int>(i),
                static_cast<VertexId>(i));
  }

  std::mt19937 rng(_seed);
  std::uniform_int_distribution<std::size_t> vDist(0, _numVertices - 1);
  std::uniform_real_distribution<double> wDist(0.1, 10.0);

  const std::size_t numEdges = static_cast<std::size_t>(
      _avgDegree * static_cast<double>(_numVertices) / 2.0);

  for (std::size_t i = 0; i < numEdges; ++i)
  {
    auto a = vDist(rng);
    auto b = vDist(rng);
    if (a == b)
      continue;
    g.AddEdge({a, b}, 0.0, wDist(rng));
  }
  return g;
}

/// \brief Build a connected chain graph: 0 - 1 - 2 - ... - (n-1).
/// Used when a benchmark needs guaranteed connectivity (e.g. Dijkstra
/// reachability).
/// \param[in] _numVertices Number of vertices in the chain.
/// \return The constructed chain graph.
template <typename EdgeT>
Graph<int, double, EdgeT> makeChainGraph(std::size_t _numVertices)
{
  Graph<int, double, EdgeT> g;
  for (std::size_t i = 0; i < _numVertices; ++i)
  {
    g.AddVertex("v" + std::to_string(i), static_cast<int>(i),
                static_cast<VertexId>(i));
  }
  for (std::size_t i = 0; i + 1 < _numVertices; ++i)
    g.AddEdge({i, i + 1}, 0.0, 1.0);
  return g;
}

constexpr double kAvgDegree = 6.0;

}  // namespace

/////////////////////////////////////////////////
static void BM_BFS_Random(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = BreadthFirstSort(g, 0);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_BFS_Random)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_DFS_Random(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = DepthFirstSort(g, 0);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_DFS_Random)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_Dijkstra_Chain(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeChainGraph<UndirectedEdge<double>>(n);
  for (auto _ : _state)
  {
    auto r = Dijkstra(g, 0);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_Dijkstra_Chain)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_Dijkstra_Random(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = Dijkstra(g, 0);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_Dijkstra_Random)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_ConnectedComponents(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = ConnectedComponents(g);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_ConnectedComponents)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_BFS_Chain(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeChainGraph<UndirectedEdge<double>>(n);
  for (auto _ : _state)
  {
    auto r = BreadthFirstSort(g, 0);
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_BFS_Chain)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_AccessorVertices(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = g.Vertices().size();
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_AccessorVertices)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
static void BM_AccessorEdges(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    auto r = g.Edges().size();
    benchmark::DoNotOptimize(r);
  }
}
BENCHMARK(BM_AccessorEdges)->RangeMultiplier(10)->Range(100, 10000);

/////////////////////////////////////////////////
// Iterates Vertices() with a real loop body (sums payloads). This is the
// shape of code an actual user would write; stresses both the accessor
// and downstream iteration cost.
static void BM_AccessorVerticesIterateAndSum(benchmark::State &_state)
{
  const auto n = static_cast<std::size_t>(_state.range(0));
  auto g = makeRandomGraph<UndirectedEdge<double>>(n, kAvgDegree);
  for (auto _ : _state)
  {
    int64_t total = 0;
    for (auto const &v : g.Vertices())
      total += v.second.get().Data();
    benchmark::DoNotOptimize(total);
  }
}
BENCHMARK(BM_AccessorVerticesIterateAndSum)
    ->RangeMultiplier(10)->Range(100, 10000);

BENCHMARK_MAIN();
