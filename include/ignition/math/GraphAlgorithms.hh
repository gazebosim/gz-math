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
#ifndef IGNITION_MATH_GRAPHALGORITHMS_HH_
#define IGNITION_MATH_GRAPHALGORITHMS_HH_

#include <algorithm>
#include <list>
#include <map>
#include <stack>
#include <vector>

#include "ignition/math/Graph.hh"
#include "ignition/math/Helpers.hh"

namespace ignition
{
  namespace math
  {
    /// \brief Depth first search (DFS).
    /// Starting from the node == _from, it traverses the graph as far as
    /// possible along each branch before backtracking.
    /// \param[in] _graph A graph.
    /// \param[in] _from The starting vertex.
    /// \return The vector of vertices Ids traversed.
    template<typename V, typename E, typename EdgeType>
    std::vector<VertexId> DFS(const Graph<V, E, EdgeType> &_graph,
                              const VertexId &_from)
    {
      std::vector<VertexId> visited;
      std::stack<VertexId> pending({_from});

      while (!pending.empty())
      {
        auto v = pending.top();
        pending.pop();

        // The vertex hasn't been visited yet.
        if (std::find(visited.begin(), visited.end(), v) == visited.end())
          visited.push_back(v);

        // Add more vertices to visit if they haven't been visited yet.
        auto adjacents = _graph.AdjacentsFrom(v);
        for (auto const &adj : adjacents)
        {
          v = adj.first;
          if (std::find(visited.begin(), visited.end(), v) == visited.end())
            pending.push(v);
        }
      }

      return visited;
    }

    /// \brief Breadth first search (BFS).
    /// Starting from the node == _from, it traverses the graph exploring the
    /// neighbors first, before moving to the next level neighbors.
    /// \param[in] _graph A graph.
    /// \param[in] _from The starting vertex.
    /// \return The vector of vertices Ids traversed.
    template<typename V, typename E, typename EdgeType>
    std::vector<VertexId> BFS(const Graph<V, E, EdgeType> &_graph,
                              const VertexId &_from)
    {
      std::vector<VertexId> visited;
      std::list<VertexId> pending = {_from};

      while (!pending.empty())
      {
        auto v = pending.front();
        pending.pop_front();

        // The vertex hasn't been visited yet.
        if (std::find(visited.begin(), visited.end(), v) == visited.end())
          visited.push_back(v);

        // Add more vertices to visit if they haven't been visited yet.
        auto adjacents = _graph.AdjacentsFrom(v);
        for (auto const &adj : adjacents)
        {
          v = adj.first;
          if (std::find(visited.begin(), visited.end(), v) == visited.end())
            pending.push_back(v);
        }
      }

      return visited;
    }

    /// \brief Dijkstra algorithm.
    /// Find the shortest path between a pair of vertices.
    /// \param[in] _graph A graph.
    /// \param[in] _from The starting vertex.
    /// \param[in] _to The destination vertex.
    /// \return The vector of vertex Ids contained in the path or an empty
    /// vector if there's no path.
    template<typename V, typename E, typename EdgeType>
    std::vector<VertexId> dijkstra(const Graph<V, E, EdgeType> &_graph,
                                   const VertexId &_from,
                                   const VertexId &_to)
    {
      auto allVertices = _graph.Vertices();

      // Sanity check: The source and destination vertices should exist.
      for (auto const &v : {_from, _to})
      {
        if (allVertices.find(v) == allVertices.end())
        {
          std::cerr << "Vertex [" << v << "] Not found" << std::endl;
          return {};
        }
      }

      // Initialization
      std::map<VertexId, double> dist;
      std::map<VertexId, VertexId> prev;
      std::vector<VertexId> unvisited;

      for (auto const &v : allVertices)
      {
        auto id = v.first;
        unvisited.push_back(id);
        dist[id] = MAX_D;
        prev[id] = id;
      }

      // Distance of source vertex from itself is always 0.
      dist[_from] = 0;

      while (!unvisited.empty() &&
             std::find(unvisited.begin(), unvisited.end(), _to) !=
               unvisited.end())
      {
        // Get the next vertex from the unvisited list.
        double minDist = MAX_D;
        VertexId id = kNullId;
        for (auto const &unvisitedId : unvisited)
        {
          if (dist.at(unvisitedId) < minDist)
          {
            minDist = dist.at(unvisitedId);
            id = unvisitedId;
          }
        }
        if (id == kNullId)
         return {};

        // Update unvisited.
        unvisited.erase(
          std::remove(unvisited.begin(), unvisited.end(), id), unvisited.end());

        for (auto const &edgePair : _graph.IncidentsFrom(id))
        {
          const auto &edgeId = edgePair.first;
          const auto &edge = _graph.EdgeFromId(edgeId);
          const auto &neighborId = edge.From(id);
          if (neighborId == kNullId)
            continue;

          double weight = edge.Weight();
          if (std::find(unvisited.begin(), unvisited.end(), neighborId) !=
                unvisited.end())
          {
            if ((dist.at(neighborId) == MAX_D) ||
                (dist.at(id) + weight < dist.at(neighborId)))
            {
              dist.at(neighborId) = dist.at(id) + weight;
              prev[neighborId] = id;
            }
          }
        }
      }

      if (prev.at(_to) == kNullId)
        return {};

      // Populate the result.
      std::vector<VertexId> res = {_to};
      auto next = _to;
      while (next != _from)
      {
        next = prev.at(next);
        res.insert(res.begin(), next);
      }
      return res;
    }
  }
}
#endif
