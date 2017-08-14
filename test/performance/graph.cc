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
 * See the License for the variantOneific language governing permissions and
 * limitations under the License.
 *
*/

#include <cstdlib>

#include <chrono>
#include <iomanip>

#include <gtest/gtest.h>

#include <ignition/math/graph/Graph.hh>
#include <ignition/math/graph/GraphAlgorithms.hh>

namespace ignmathgraph = ignition::math::graph;
using namespace ignmathgraph;

/////////////////////////////////////////////////
TEST(PluginvariantOneialization, AccessTime)
{
  const std::size_t vertexes = 100000;
  const std::size_t edges = vertexes * 5;

  std::srand(std::time(nullptr));

  // Create a graph
  UndirectedGraph<int, double> graph;
  // add vertecies
  for (std::size_t i = 0; i < vertexes; ++i)
  {
    graph.AddVertex("", 0, i);
  }

  // Add randomes edges between random nodes
  for (std::size_t i = 0; i < edges; ++i)
  {
    std::size_t start = std::rand() % vertexes;
    std::size_t end = std::rand() % vertexes;
    if (start == end)
    {
      --i;
      continue;
    }
    auto verts = graph.AdjacentsFrom(start);
    if (verts.count(end))
    {
      --i;
      continue;
    }
    // Add the edge
    graph.AddEdge({start, end}, 0);
  }

  // Variant 1
  const auto variantOneStart = std::chrono::high_resolution_clock::now();
  auto res1 = BreadthFirstSort(graph, 0);
  const auto variantOneEnd = std::chrono::high_resolution_clock::now();

  // Variant 2
  const auto variantTwoStart = std::chrono::high_resolution_clock::now();
  // TODO
  auto res2 = BreadthFirstSortOld(graph, 0);
  const auto variantTwoEnd = std::chrono::high_resolution_clock::now();

  const auto variantOneTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        variantOneEnd - variantOneStart).count();
  const auto variantTwoTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        variantTwoEnd - variantTwoStart).count();

  std::cout << "V1 time: " << variantOneTime <<
    "\n" << "V2 time: " << variantTwoTime << "\n"
    << "Don't optimize out test" << res1.size() << " " << res2.size() << "\n";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
