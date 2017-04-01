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
#ifndef IGNITION_MATH_GRAPH_HH_
#define IGNITION_MATH_GRAPH_HH_

#include <cassert>
// int64_t
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace ignition
{
  namespace math
  {
    /// \def VertexId.
    /// \brief The unique Id of each vertex.
    using VertexId = int64_t;

    /// \brief Represents an invalid Id.
    static const VertexId kNullId = std::numeric_limits<VertexId>::min();

    /// \brief A vertex of a graph. It stores user information and keeps
    /// an internal unique Id.
    template<typename V>
    class Vertex
    {
      /// \brief An invalid vertex.
      public: static Vertex<V> NullVertex;

      /// \brief Constructor.
      /// \param[in] _data User information.
      /// \param[in] _name Non-unique vertex name.
      /// \param[in] _id Optional unique id.
      public: Vertex(const V &_data,
                     const std::string &_name,
                     const VertexId _id = kNullId)
        : data(_data),
          name(_name),
          id(_id)
      {
      }

      /// \brief Retrieve the user information.
      /// \return Reference to the user information.
      public: const V &Data() const
      {
        return this->data;
      }

      /// \brief Get the vertex Id.
      /// \return The vertex Id.
      public: VertexId Id() const
      {
        return this->id;
      }

      /// \brief Get the vertex name.
      /// \return The vertex name.
      public: std::string Name() const
      {
        return this->name;
      }

      /// \brief Whether the vertex is considered valid or not.
      /// \return True when the vertex is valid or false otherwise (invalid Id).
      public: bool Valid() const
      {
        return this->id != kNullId;
      }

      /// \brief User information.
      private: V data;

      /// \brief Non-unique vertex name.
      private: std::string name = "";

      /// \brief Unique vertex Id.
      private: VertexId id = kNullId;
    };

    /// \brief An invalid vertex.
    template<typename V>
    Vertex<V> Vertex<V>::NullVertex(V(), "__null__", kNullId);

    /// \def VertexId_S
    /// \brief Set of vertex Ids.
    using VertexId_S = std::set<VertexId>;

    /// \def VertexRef_M
    /// \brief Map of vertices. The key is the vertex Id. The value is a
    /// reference to the vertex.
    template<typename V>
    using VertexRef_M =
      std::map<VertexId, std::reference_wrapper<const Vertex<V>>>;

    /// \def EdgeId.
    /// \brief The unique Id for an edge.
    using EdgeId = int64_t;

    /// \brief Generic edge class. An edge has two ends and some constraint
    /// between them. For example, a directed edge only allows traversing the
    /// edge in one direction.
    class Edge
    {
      /// \brief Constructor.
      /// \param[in] _id Unique id.
      public: Edge(const EdgeId _id)
        : id(_id)
      {
      }

      /// \brief Get the edge Id.
      /// \return The edge Id.
      public: EdgeId Id() const
      {
        return this->id;
      }

      /// \brief Get the destination end that is reachable from a source end of
      /// an edge.
      ///
      /// E.g.: Let's assume that we have an undirected edge (e1) with ends
      /// (v1) and (v2). The operation e1.From(v1) returns (v2).
      /// The operation e1.From(v2) returns (v1).
      ///
      /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
      /// (v1) and the head end (v2). The operation e2.From(v1) returns (v2).
      /// The operation e2.From(v2) returns nullptr.
      ///
      /// \param[in] _from Source vertex.
      /// \return The other vertex of the edge reachable from the "_from"
      /// vertex or NullVertex otherwise.
      public: virtual VertexId From(const VertexId &_from) const = 0;

      /// \brief Get if the edge is valid. An edge is valid if its linked in a
      /// graph and its vertices are reachable.
      /// \return True when the edge is valid.
      /// \sa SetValid
      public: bool Valid() const
      {
        return this->valid && this->id != kNullId;
      }

      /// \brief Set an edge valid or invalid.
      /// \param[in] _newValue Validity.
      /// \sa Valid
      public: void SetValid(const bool _newValue)
      {
        this->valid = _newValue;
      }

      /// \brief Unique edge Id.
      private: EdgeId id = kNullId;

      /// \brief True when the edge is connected in a graph or false otherwise.
      /// This member variable exists to prevent the following situation:
      /// Imagine that you have a shared pointer pointing to a valid edge
      /// connected in a graph. Now, you use the Graph API and remove the
      /// edge. This operation will disconnect the edge from the graph but the
      /// edge won't be deallocated because you will keep a shared pointer.
      /// Having a shared pointer to the edge will allow you to traverse the
      /// edge and go to any of the vertices of the graph and modify it.
      /// Once an edge is removed, this flag is set to false and this will
      /// prevent you from traversing the edge and reach the vertices.
      private: bool valid = false;
    };

    /// \def EdgeId_S
    /// \brief A set of edge Ids.
    using EdgeId_S = std::set<EdgeId>;

    /// \def EdgeRef_M
    /// \brief A map of edges. The key is the edge Id. The value is a reference
    /// to the edge.
    template<typename EdgeType>
    using EdgeRef_M = std::map<EdgeId, std::reference_wrapper<const EdgeType>>;

    /// \brief A generic graph class.
    /// Both vertices and edges can store user information. A vertex could be
    /// created passing a custom Id if needed, otherwise it will be choosen
    /// internally. The vertices also have a name that could be reused among
    /// other vertices if needed. This class supports the use of different edge
    /// types (e.g. directed or undirected edges).
    template<typename V, typename E, typename EdgeType>
    class Graph
    {
      /// \brief Add a new vertex to the graph.
      /// \param[in] _data Data to be stored in the vertex.
      /// \param[in] _name Name of the vertex. It doesn't have to be unique.
      /// \param[in] _id Optional Id to be used for this vertex.
      public: Vertex<V> &AddVertex(const V &_data,
                                   const std::string &_name,
                                   const VertexId &_id = kNullId)
      {
        auto id = _id;
        // The user didn't provide an Id, we generate it.
        if (id == kNullId)
          id = this->NextVertexId();
        // The user provided an Id but already exists.
        else if (this->vertices.find(id) != this->vertices.end())
          return Vertex<V>::NullVertex;

        // Create the vertex.
        this->vertices.insert(std::make_pair(id, Vertex<V>(_data, _name, id)));

        // Link the vertex with an empty list of edges.
        this->adjList[id] = EdgeId_S();

        // Update the map of names.
        this->names.insert(std::make_pair(_name, id));

        return this->vertices.at(id);
      }

      /// \brief The collection of all vertices in the graph.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> Vertices() const
      {
        VertexRef_M<V> res;
        for (auto const &v : this->vertices)
          res.emplace(std::make_pair(v.first, std::cref(v.second)));

        return std::move(res);
      }

      /// \brief The collection of all vertices in the graph with name == _name.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> Vertices(const std::string &_name) const
      {
        VertexRef_M<V> res;
        for (auto &vertex : this->vertices)
        {
          if (vertex.second.Name() == _name)
            res.emplace(std::make_pair(vertex.first, std::cref(vertex.second)));
        }

        return std::move(res);
      }

      /// \brief Links an edge to the graph.
      /// \param[in] _edge A new edge.
      /// \return A reference to the new link created.
      public: EdgeType &LinkEdge(EdgeType &&_edge)
      {
        _edge.SetValid(true);

        auto vertices = _edge.Vertices();
        if (vertices.size() != 2u)
        {
          _edge.SetValid(false);
          return EdgeType::NullEdge;
        }

        // Sanity check: Both vertices should exist.
        for (auto const &v : vertices)
        {
          auto itV = this->vertices.find(v);
          if (itV == this->vertices.end())
          {
            _edge.SetValid(false);
            return EdgeType::NullEdge;
          }
        }

        // Link the new edge.
        for (auto const &v : vertices)
        {
          if (_edge.From(v) != kNullId)
          {
            auto vertexIt = this->adjList.find(v);
            assert(vertexIt != this->adjList.end());
            vertexIt->second.insert(_edge.Id());
          }
        }

        this->edges.insert(std::make_pair(_edge.Id(), std::move(_edge)));

        return this->edges.at(_edge.Id());
      }

      /// \brief The collection of all edges in the graph.
      /// \return A map of edges, where keys are Ids and values are references
      /// to the edges.
      public: EdgeRef_M<EdgeType> Edges() const
      {
        EdgeRef_M<EdgeType> res;
        for (auto &edge : this->edges)
        {
          res.emplace(std::make_pair(edge.first, std::cref(edge.second)));
        }

        return std::move(res);
      }

      /// \brief Get all neighbors vertices that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The Id of the vertex to check adjacents.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> Adjacents(const VertexId &_vertex) const
      {
        VertexRef_M<V> res;

        auto vertexIt = this->adjList.find(_vertex);
        if (vertexIt == this->adjList.end())
          return res;

        for (auto const &edgeId : vertexIt->second)
        {
          auto edge = this->EdgeFromId(edgeId);
          auto neighborVertexId = edge.From(_vertex);
          auto neighbotVertex = this->VertexFromId(neighborVertexId);
          res.emplace(
            std::make_pair(neighborVertexId, std::cref(neighbotVertex)));
        }

        return res;
      }

      /// \brief Get all neighbors vertices that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The vertex to check adjacents.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> Adjacents(const Vertex<V> &_vertex) const
      {
        return this->Adjacents(_vertex.Id());
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Id of the vertex.
      /// \return A map of edges, where keys are Ids and values are
      /// references to the edges.
      public: EdgeRef_M<EdgeType> Incidents(const VertexId &_vertex) const
      {
        EdgeRef_M<EdgeType> res;

        auto vertexIt = this->adjList.find(_vertex);
        if (vertexIt == this->adjList.end())
          return res;

        for (auto const &nodeAdjList : this->adjList)
        {
          auto edgeIds = nodeAdjList.second;
          for (auto const &edgeId : edgeIds)
          {
            auto edge = this->EdgeFromId(edgeId);
            if (edge.From(nodeAdjList.first) == _vertex)
              res.emplace(std::make_pair(edge.Id(), std::cref(edge)));
          }
        }

        return std::move(res);
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex The vertex.
      /// \return A map of edges, where keys are Ids and values are
      /// references to the edges.
      public: EdgeRef_M<EdgeType> Incidents(const Vertex<V> &_vertex) const
      {
        return this->Incidents(_vertex.Id());
      }

      /// \brief Whether the graph is empty.
      /// \return True when there are no vertices in the graph or
      /// false otherwise.
      public: bool Empty() const
      {
        return this->vertices.empty();
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex Id of the vertex to be removed.
      /// \return True when the vertex was removed or false otherwise.
      public: bool RemoveVertex(const VertexId &_vertex)
      {
        auto vIt = this->vertices.find(_vertex);
        if (vIt == this->vertices.end())
          return false;

        std::string name = vIt->second.Name();

        // Remove incident edges.
        auto incidents = this->Incidents(_vertex);
        for (auto edgePair : incidents)
          this->RemoveEdge(edgePair.first);

        // Remove all outgoing edges.
        auto adjIt = this->adjList.find(_vertex);
        if (adjIt != this->adjList.end())
        {
          auto &edgeIds = adjIt->second;
          for (auto &edge : edgeIds)
            this->RemoveEdge(edge);
        }
        this->adjList.erase(_vertex);

        // Remove the vertex.
        this->vertices.erase(_vertex);

        // Get an iterator to all vertices sharing name.
        auto iterPair = this->names.equal_range(name);
        for (auto it = iterPair.first; it != iterPair.second; ++it)
        {
          if (it->second == _vertex)
          {
            this->names.erase(it);
            break;
          }
        }

        return true;
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex The vertex to be removed.
      /// \return True when the vertex was removed or false otherwise.
      public: bool RemoveVertex(Vertex<V> &_vertex)
      {
        return this->RemoveVertex(_vertex.Id());
      }

      /// \brief Remove all vertices with name == _name.
      /// \param[in] _name Name of the vertices to be removed.
      /// \return The number of vertices removed.
      public: size_t RemoveVertices(const std::string &_name)
      {
        size_t n = this->names.count(_name);
        for (auto i = 0; i < n; ++i)
        {
          auto iter = this->names.find(_name);
          this->RemoveVertex(iter->second);
        }

        return n;
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertices from the edge.
      /// \param[in] _edge Id of the edge to be removed.
      /// \return True when the edge was removed or false otherwise.
      public: bool RemoveEdge(const EdgeId &_edge)
      {
        auto edgeIt = this->edges.find(_edge);
        if (edgeIt == this->edges.end())
          return false;

        auto vertices = edgeIt->second.Vertices();

        // Unlink the edge.
        for (auto const &v : vertices)
        {
          if (edgeIt->second.From(v) != kNullId)
          {
            auto vertex = this->adjList.find(v);
            assert(vertex != this->adjList.end());
            vertex->second.erase(_edge);
          }
        }

        this->edges.erase(_edge);

        // Mark the edge as invalid. This will prevent to reach any vertices if
        // there are any shared pointers keeping the edge alive.
        // _edge->SetValid(false);

        return true;
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertices from the edge.
      /// \param[in] _edge The edge to be removed.
      /// \return True when the edge was removed or false otherwise.
      public: bool RemoveEdge(EdgeType &_edge)
      {
        return this->RemoveEdge(_edge.Id());
      }

      /// \brief Get a reference to a vertex using its Id.
      /// \param[in] _id The Id of the vertex.
      /// \return A reference to the vertex with Id = _id or NullVertex if
      /// not found.
      public: const Vertex<V> &VertexFromId(const VertexId &_id) const
      {
        auto iter = this->vertices.find(_id);
        if (iter == this->vertices.end())
          return Vertex<V>::NullVertex;

        return iter->second;
      }

      /// \brief Get a reference to an edge using its Id.
      /// \param[in] _id The Id of the edge.
      /// \return A reference to the edge with Id = _id or NullEdge if
      /// not found.
      public: const EdgeType &EdgeFromId(const EdgeId &_id) const
      {
        auto iter = this->edges.find(_id);
        if (iter == this->edges.end())
          return EdgeType::NullEdge;

        return iter->second;
      }

      /// \brief Get an available Id to be assigned to a new vertex.
      /// \return The next available Id.
      private: VertexId &NextVertexId()
      {
        while (this->vertices.find(this->nextVertexId) != this->vertices.end())
          ++this->nextVertexId;

        return this->nextVertexId;
      }

      /// \brief Get an available Id to be assigned to a new edge.
      /// \return The next available Id.
      protected: VertexId &NextEdgeId()
      {
        while (this->edges.find(this->nextEdgeId) != this->edges.end())
          ++this->nextEdgeId;

        return this->nextEdgeId;
      }

      /// \brief The set of vertices.
      protected: std::map<VertexId, Vertex<V>> vertices;

      /// \brief The set of edges.
      protected: std::map<EdgeId, EdgeType> edges;

      /// \brief The adjacency list.
      /// A map where the keys are vertex Ids. For each vertex (v)
      /// with id (vId), the map value contains a set of edge Ids. Each of
      /// the edges (e) with Id (eId) represents a connected path from (v) to
      /// another vertex via (e).
      protected: std::map<VertexId, EdgeId_S> adjList;

      /// \brief Association between names and vertices curently used.
      protected: std::multimap<std::string, VertexId> names;

      /// \brief The next vertex Id to be assigned to a new vertex.
      private: VertexId nextVertexId = 0;

      /// \brief The next edge Id to be assigned to a new edge.
      private: VertexId nextEdgeId = 0;
    };
  }
}
#endif
