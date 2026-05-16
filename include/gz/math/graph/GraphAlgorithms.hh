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
#ifndef GZ_MATH_GRAPH_GRAPHALGORITHMS_HH_
#define GZ_MATH_GRAPH_GRAPHALGORITHMS_HH_

#include <functional>
#include <map>
#include <queue>
#include <stack>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/math/config.hh>
#include "gz/math/graph/Graph.hh"
#include "gz/math/Helpers.hh"

namespace ignition
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace graph
{
  /// \typedef CostInfo.
  /// \brief Used in Dijkstra. For a given source vertex, this pair represents
  /// the cost (first element) to reach a destination vertex (second element).
  using CostInfo = std::pair<double, VertexId>;

  /// \brief Breadth first sort (BFS).
  /// Starting from the vertex == _from, it traverses the graph exploring the
  /// neighbors first, before moving to the next level neighbors.
  /// \param[in] _graph A graph.
  /// \param[in] _from The starting vertex.
  /// \return The vector of vertices Ids traversed in a breadth first manner.
  /// An empty vector if _from is not a vertex of the graph.
  template<typename V, typename E, typename EdgeType>
  std::vector<VertexId> BreadthFirstSort(const Graph<V, E, EdgeType> &_graph,
                                         const VertexId &_from)
  {
    if (!_graph.VertexFromId(_from).Valid())
      return {};

    std::vector<VertexId> visited;
    std::unordered_set<VertexId> seen;
    std::queue<VertexId> pending;

    // Mark-on-enqueue: each vertex enters the queue at most once.
    pending.push(_from);
    seen.insert(_from);

    while (!pending.empty())
    {
      const VertexId u = pending.front();
      pending.pop();
      visited.push_back(u);

      for (auto const &adj : _graph.AdjacentsFrom(u))
      {
        const VertexId next = adj.first;
        if (seen.insert(next).second)
          pending.push(next);
      }
    }
    return visited;
  }

  /// \brief Depth first sort (DFS).
  /// Starting from the vertex == _from, it visits the graph as far as
  /// possible along each branch before backtracking.
  /// \param[in] _graph A graph.
  /// \param[in] _from The starting vertex.
  /// \return The vector of vertices Ids visited in a depth first manner.
  /// An empty vector if _from is not a vertex of the graph.
  template<typename V, typename E, typename EdgeType>
  std::vector<VertexId> DepthFirstSort(const Graph<V, E, EdgeType> &_graph,
                                       const VertexId &_from)
  {
    if (!_graph.VertexFromId(_from).Valid())
      return {};

    std::vector<VertexId> visited;
    std::unordered_set<VertexId> seen;
    std::stack<VertexId> pending;
    pending.push(_from);

    // Mark-on-pop: matches the textbook DFS visitation order. Children are
    // pushed unconditionally and duplicate entries are skipped at pop time.
    while (!pending.empty())
    {
      const VertexId u = pending.top();
      pending.pop();

      if (!seen.insert(u).second)
        continue;
      visited.push_back(u);

      for (auto const &adj : _graph.AdjacentsFrom(u))
      {
        const VertexId next = adj.first;
        if (!seen.count(next))
          pending.push(next);
      }
    }
    return visited;
  }

  /// \brief Dijkstra algorithm.
  /// Find the shortest path between the vertices in a graph.
  /// If only a graph and a source vertex is provided, the algorithm will
  /// find shortest paths from the source vertex to all other vertices in the
  /// graph. If an additional destination vertex is provided, the algorithm
  /// will stop when the shortest path is found between the source and
  /// destination vertex.
  /// \param[in] _graph A graph.
  /// \param[in] _from The starting vertex.
  /// \param[in] _to Optional destination vertex.
  /// \return A map where the keys are the destination vertices. For each
  /// destination, the value is another pair, where the key is the shortest
  /// cost from the origin vertex. The value is the previous neighbor Id in the
  /// shortest path.
  /// Note: In the case of providing a destination vertex, only the entry in the
  /// map with key = _to should be used. The rest of the map may contain
  /// incomplete information. If you want all shortest paths to all other
  /// vertices, please remove the destination vertex.
  /// If the source or destination vertex don't exist, the function will return
  /// an empty map.
  ///
  /// E.g.: Given the following undirected graph, g, with five vertices:
  ///
  ///              (6)                |
  ///           0-------1             |
  ///           |      /|\            |
  ///           |     / | \(5)        |
  ///           | (2)/  |  \          |
  ///           |   /   |   2         |
  ///        (1)|  / (2)|  /          |
  ///           | /     | /(5)        |
  ///           |/      |/            |
  ///           3-------4             |
  ///              (1)                |
  ///
  /// This is the resut of Dijkstra(g, 0):
  ///
  /// \code
  /// ================================
  /// | Dst | Cost | Previous vertex |
  /// ================================
  /// |  0  |  0   |        0        |
  /// |  1  |  3   |        3        |
  /// |  2  |  7   |        4        |
  /// |  3  |  1   |        0        |
  /// |  4  |  2   |        3        |
  /// ================================
  /// \endcode
  ///
  /// This is the result of Dijkstra(g, 0, 3):
  ///
  /// \code
  /// ================================
  /// | Dst | Cost | Previous vertex |
  /// ================================
  /// |  0  |  0   |        0        |
  /// |  1  |ignore|     ignore      |
  /// |  2  |ignore|     ignore      |
  /// |  3  |  1   |        0        |
  /// |  4  |ignore|     ignore      |
  /// ================================
  /// \endcode
  ///
  template<typename V, typename E, typename EdgeType>
  std::map<VertexId, CostInfo> Dijkstra(const Graph<V, E, EdgeType> &_graph,
                                        const VertexId &_from,
                                        const VertexId &_to = kNullId)
  {
    auto allVertices = _graph.Vertices();

    // Sanity check: The source vertex should exist.
    if (allVertices.find(_from) == allVertices.end())
    {
      std::cerr << "Vertex [" << _from << "] Not found" << std::endl;
      return {};
    }

    // Sanity check: The destination vertex should exist (if used).
    if (_to != kNullId &&
        allVertices.find(_to) == allVertices.end())
    {
      std::cerr << "Vertex [" << _to << "] Not found" << std::endl;
      return {};
    }

    // Store vertices that are being preprocessed.
    std::priority_queue<CostInfo,
      std::vector<CostInfo>, std::greater<CostInfo>> pq;

    // Create a map for distances and next neightbor and initialize all
    // distances as infinite.
    std::map<VertexId, CostInfo> dist;
    for (auto const &v : allVertices)
    {
      auto id = v.first;
      dist[id] = std::make_pair(MAX_D, kNullId);
    }

    // Insert _from in the priority queue and initialize its distance as 0.
    pq.push(std::make_pair(0.0, _from));
    dist[_from] = std::make_pair(0.0, _from);

    while (!pq.empty())
    {
      // This is the minimum distance vertex.
      const double poppedCost = pq.top().first;
      VertexId u = pq.top().second;

      // Shortcut: Destination vertex found, exiting.
      if (_to != kNullId && _to == u)
        break;

      pq.pop();

      // Skip stale priority-queue entries left behind by relaxation
      // updates: dist[u] is the authoritative cost; if the popped cost is
      // greater, this entry was queued before u was settled.
      if (poppedCost > dist[u].first)
        continue;

      for (auto const &edgePair : _graph.IncidentsFrom(u))
      {
        const auto &edge = edgePair.second.get();
        const auto &v = edge.From(u);
        double weight = edge.Weight();

        // If there is a shorter path to v through u.
        if (dist[v].first > dist[u].first + weight)
        {
          // Update distance of v.
          dist[v] = std::make_pair(dist[u].first + weight, u);
          pq.push(std::make_pair(dist[v].first, v));
        }
      }
    }

    return dist;
  }

  /// \brief Calculate the connected components of an undirected graph.
  /// A connected component of an undirected graph is a subgraph in which any
  /// two vertices are connected to each other by paths, and which is connected
  /// to no additional vertices in the supergraph.
  /// \sa https://en.wikipedia.org/wiki/Connected_component_(graph_theory)
  /// \param[in] _graph A graph.
  /// \return A vector of graphs. Each element of the graph is a component
  /// (subgraph) of the original graph.
  template<typename V, typename E>
  std::vector<UndirectedGraph<V, E>> ConnectedComponents(
    const UndirectedGraph<V, E> &_graph)
  {
    std::map<VertexId, unsigned int> visited;
    unsigned int componentCount = 0;

    for (auto const &v : _graph.Vertices())
    {
      if (visited.find(v.first) == visited.end())
      {
        auto component = BreadthFirstSort(_graph, v.first);
        for (auto const &vId : component)
          visited[vId] = componentCount;
        ++componentCount;
      }
    }

    std::vector<UndirectedGraph<V, E>> res(componentCount);

    // Create the vertices.
    for (auto const &vPair : _graph.Vertices())
    {
      const auto &v = vPair.second.get();
      const auto &componentId = visited[v.Id()];
      res[componentId].AddVertex(v.Name(), v.Data(), v.Id());
    }

    // Create the edges.
    for (auto const &ePair : _graph.Edges())
    {
      const auto &e = ePair.second.get();
      const auto &vertices = e.Vertices();
      const auto &componentId = visited[vertices.first];
      res[componentId].AddEdge(vertices, e.Data(), e.Weight());
    }

    return res;
  }

  /// \brief Copy a DirectedGraph to an UndirectedGraph with the same vertices
  /// and edges.
  /// \param[in] _graph A directed graph.
  /// \return An undirected graph with the same vertices and edges as the
  /// original graph.
  template<typename V, typename E>
  UndirectedGraph<V, E> ToUndirectedGraph(const DirectedGraph<V, E> &_graph)
  {
    std::vector<Vertex<V>> vertices;
    std::vector<EdgeInitializer<E>> edges;

    // Add all vertices.
    for (auto const &vPair : _graph.Vertices())
    {
      vertices.push_back(vPair.second.get());
    }

    // Add all edges.
    for (auto const &ePair : _graph.Edges())
    {
      auto const &e = ePair.second.get();
      edges.push_back({e.Vertices(), e.Data(), e.Weight()});
    }

    return UndirectedGraph<V, E>(vertices, edges);
  }

  /// \brief Walk parent edges from `_vertex` up to a root and return the
  /// chain of ancestors in walk order (immediate parent first, root last)
  /// together with a flag indicating whether the walk terminated cleanly
  /// at a root (no parent) or was aborted by the cycle guard.
  /// In a tree this returns the unique parent chain; in a more general
  /// directed graph it follows the *first* incoming edge at each step
  /// (deterministic via the adjacency-list ordering) and aborts on cycles.
  ///
  /// This subsumes the hand-coded "walk to root" loops widely seen in
  /// downstream code (e.g. gz-sim's worldPose / worldEntity / scopedName,
  /// sdformat's FrameSemantics::FindSourceVertex).
  ///
  /// \param[in] _graph Any graph (typically a directed forest/tree).
  /// \param[in] _vertex Starting vertex.
  /// \return A pair (chain, reachedRoot):
  ///   - chain: vertex ids in walk order (parent, grandparent, ..., root).
  ///     Empty if `_vertex` is invalid or has no parent.
  ///   - reachedRoot: true if the walk terminated at a root (parent chain
  ///     is complete). False if `_vertex` was invalid, or the walk was
  ///     aborted because a previously visited vertex was reached again
  ///     (cycle), in which case `chain` holds the partial path traversed
  ///     up to the revisit.
  template<typename V, typename E, typename EdgeType>
  std::pair<std::vector<VertexId>, bool> Ancestors(
      const Graph<V, E, EdgeType> &_graph, const VertexId &_vertex)
  {
    std::vector<VertexId> chain;
    if (!_graph.VertexFromId(_vertex).Valid())
      return {chain, false};

    std::unordered_set<VertexId> seen;
    seen.insert(_vertex);
    VertexId cur = _vertex;
    while (true)
    {
      auto parents = _graph.AdjacentsTo(cur);
      if (parents.empty())
        return {chain, true};
      const VertexId next = parents.begin()->first;
      // Cycle guard: stop if we revisit a vertex.
      if (!seen.insert(next).second)
        return {chain, false};
      chain.push_back(next);
      cur = next;
    }
  }

  /// \brief Test whether `_ancestor` lies on the parent chain above
  /// `_descendant`. O(depth) -- walks `_descendant` up via Ancestors() and
  /// stops as soon as a match is found. Returns false for `_a == _d`
  /// (consistent with the strict ancestor relation).
  /// \param[in] _graph Any graph.
  /// \param[in] _ancestor Candidate ancestor vertex id.
  /// \param[in] _descendant Candidate descendant vertex id.
  /// \return True if `_ancestor` is on the parent chain of `_descendant`.
  template<typename V, typename E, typename EdgeType>
  bool IsAncestor(
      const Graph<V, E, EdgeType> &_graph,
      const VertexId &_ancestor,
      const VertexId &_descendant)
  {
    if (_ancestor == _descendant)
      return false;
    if (!_graph.VertexFromId(_ancestor).Valid() ||
        !_graph.VertexFromId(_descendant).Valid())
    {
      return false;
    }

    std::unordered_set<VertexId> seen;
    seen.insert(_descendant);
    VertexId cur = _descendant;
    while (true)
    {
      auto parents = _graph.AdjacentsTo(cur);
      if (parents.empty())
        return false;
      const VertexId next = parents.begin()->first;
      if (next == _ancestor)
        return true;
      // Cycle guard.
      if (!seen.insert(next).second)
        return false;
      cur = next;
    }
  }

  /// \brief Lowest common ancestor of two vertices in a directed forest.
  /// Walks `_a` up to root collecting ancestors, then walks `_b` up until
  /// hitting one of those ancestors. O(depth_a + depth_b).
  ///
  /// Useful for relative-frame computation: if two entities live in the
  /// same world tree, the LCA is the deepest shared frame, and a relative
  /// pose can be composed by chaining transforms a->LCA and LCA->b.
  ///
  /// \param[in] _graph A directed graph (expected to be a forest).
  /// \param[in] _a One vertex.
  /// \param[in] _b Another vertex.
  /// \return The LCA vertex id, or kNullId if the two vertices share no
  /// common ancestor (different trees) or either is invalid. If `_a == _b`,
  /// returns `_a`.
  template<typename V, typename E, typename EdgeType>
  VertexId LowestCommonAncestor(
      const Graph<V, E, EdgeType> &_graph,
      const VertexId &_a, const VertexId &_b)
  {
    if (!_graph.VertexFromId(_a).Valid() ||
        !_graph.VertexFromId(_b).Valid())
    {
      return kNullId;
    }
    if (_a == _b)
      return _a;

    std::unordered_set<VertexId> ancestorsA;
    ancestorsA.insert(_a);
    for (auto v : Ancestors(_graph, _a).first)
      ancestorsA.insert(v);

    if (ancestorsA.count(_b))
      return _b;
    for (auto v : Ancestors(_graph, _b).first)
    {
      if (ancestorsA.count(v))
        return v;
    }
    return kNullId;
  }

  /// \brief Extract the subgraph induced by `_root` and all descendants
  /// reachable from it. Vertices and edges are copied (preserving ids,
  /// names, data, weights). All edges whose endpoints both lie in the
  /// reachable set are kept, including back-edges -- so if the source
  /// graph contains cycles within the reachable region, the cycles are
  /// preserved in the result. Equivalent to "save just this entity and
  /// everything beneath it" -- the shape of operation gz-sim and sdformat
  /// would invoke when serializing a single model out of a world.
  ///
  /// \param[in] _graph Source graph.
  /// \param[in] _root Root vertex of the subgraph.
  /// \return A new graph containing `_root` and all descendants reachable
  /// from it, plus the edges between them. Empty graph if `_root` is invalid.
  template<typename V, typename E, typename EdgeType>
  Graph<V, E, EdgeType> Subgraph(
      const Graph<V, E, EdgeType> &_graph, const VertexId &_root)
  {
    Graph<V, E, EdgeType> out;
    if (!_graph.VertexFromId(_root).Valid())
      return out;

    auto descendants = BreadthFirstSort(_graph, _root);
    std::unordered_set<VertexId> set(descendants.begin(), descendants.end());

    // Copy vertices preserving ids.
    for (auto id : descendants)
    {
      const auto &v = _graph.VertexFromId(id);
      out.AddVertex(v.Name(), v.Data(), v.Id());
    }
    // Copy edges whose endpoints both lie in the reachable set.
    for (auto const &ePair : _graph.Edges())
    {
      auto const &e = ePair.second.get();
      auto vs = e.Vertices();
      if (set.count(vs.first) && set.count(vs.second))
        out.AddEdge(vs, e.Data(), e.Weight());
    }
    return out;
  }

  /// \brief Set of all descendants of `_vertex` (including `_vertex`
  /// itself). Equivalent to BreadthFirstSort + insert-into-set, but avoids
  /// the intermediate vector copy. Matches the return shape downstream
  /// consumers (gz-sim's EntityComponentManager::Descendants) use.
  /// \param[in] _graph Source graph.
  /// \param[in] _vertex Root vertex.
  /// \return Set of vertex ids reachable from `_vertex`.
  template<typename V, typename E, typename EdgeType>
  std::unordered_set<VertexId> DescendantsSet(
      const Graph<V, E, EdgeType> &_graph, const VertexId &_vertex)
  {
    std::unordered_set<VertexId> out;
    if (!_graph.VertexFromId(_vertex).Valid())
      return out;

    std::queue<VertexId> pending;
    pending.push(_vertex);
    out.insert(_vertex);
    while (!pending.empty())
    {
      const VertexId u = pending.front();
      pending.pop();
      for (auto const &adj : _graph.AdjacentsFrom(u))
      {
        if (out.insert(adj.first).second)
          pending.push(adj.first);
      }
    }
    return out;
  }
}  // namespace graph
}  // namespace IGNITION_MATH_VERSION_NAMESPACE
}  // namespace math
}  // namespace ignition
#endif   // GZ_MATH_GRAPH_GRAPHALGORITHMS_HH_
