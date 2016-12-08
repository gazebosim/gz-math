/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <ignition/math/Graph.hh>

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(GraphTest, Constructor)
{
  Graph<int, double> graph;
  if (graph.Empty())
    std::cout << "Graph is empty\n" << std::endl;

  // Create some vertexes.
  auto v1 = graph.AddVertex(1);
  auto v2 = graph.AddVertex(3);
  auto v3 = graph.AddVertex(5);

  // List of vertexes.
  std::cout << "--Vertexes--" << std::endl;
  auto vertexes = graph.Vertexes();
  for (auto &v: vertexes)
    std::cout << v->Data() << std::endl;
  std::cout << std::endl;

  // Create and edge from node #1 to node #2.
  std::cout << "Edge #1:" << std::endl;
  auto e1 = graph.AddEdge(v1, v2, 2.0);
  if (e1)
  {
    std::cout << "  From: " << e1->Source()->Data() << std::endl;
    std::cout << "  To: " << e1->Destination()->Data() << std::endl;
    std::cout << "  Data: " << e1->Data() << std::endl << std::endl;
  }

  // Create and edge from node #2 to node #3.
  std::cout << "Edge #2:" << std::endl;
  auto e2 = graph.AddEdge(v2, v3, 3.0);
  if (e2)
  {
    std::cout << "  From: " << e2->Source()->Data() << std::endl;
    std::cout << "  To: " << e2->Destination()->Data() << std::endl;
    std::cout << "  Data: " << e2->Data() << std::endl << std::endl;
  }

  // Create and invalid edge.
  auto eInvalid = graph.AddEdge(v2, nullptr, 4.0);
  if (!eInvalid)
    std::cerr << "Invalid edge" << std::endl << std::endl;

  // List of edges.
  std::cout << "--Edges--" << std::endl;
  auto edges = graph.Edges();
  for (auto &e: edges)
  {
    std::cout << "  From: " << e->Source()->Data() << std::endl;
    std::cout << "  To: " << e->Destination()->Data() << std::endl;
    std::cout << "  Data: " << e->Data() << std::endl << std::endl;
  }

  if (!graph.Empty())
    std::cout << "Graph not empty" << std::endl << std::endl;

  std::cout << "-- Adjacents to node #1 --" << std::endl;
  auto adjacentsV1 = graph.Adjacents(v1);
  for (auto &v: adjacentsV1)
    std::cout << v->Data() << std::endl;
  std::cout << std::endl;

  std::cout << "-- Incidents to node #3 --" << std::endl;
  auto incidentsV3 = graph.Incidents(v3);
  for (auto &e: incidentsV3)
  {
    std::cout << "  From: " << e->Source()->Data() << std::endl;
    std::cout << "  To: " << e->Destination()->Data() << std::endl;
    std::cout << "  Data: " << e->Data() << std::endl << std::endl;
  }
  std::cout << std::endl;

  std::cout << graph << std::endl;

  std::cout << "-- Removing edge from node #1 to node #2 --" << std::endl;
  std::cout << std::endl;
  graph.RemoveEdge(e1);

  std::cout << graph << std::endl;

  std::cout << "-- Removing vertex #3 --" << std::endl;
  std::cout << std::endl;
  graph.RemoveVertex(v3);

  std::cout << graph << std::endl;
}
