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
#include <iostream>
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

      /// \brief Stream insertion operator. The output uses DOT graph
      /// description language.
      /// \param[out] _out The output stream.
      /// \param[in] _v Vertex to write to the stream.
      /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const Vertex<V> &_v)
      {
        _out << "  " << _v.Id() << " [label=\"" << _v.Name()
             << " (" << _v.Id() << ")\"];" << std::endl;
        return _out;
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
      public: Edge(const EdgeId &_id)
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
      /// (v1) and (v2): (v1)--(v2). The operation e1.From(v1) returns (v2).
      /// The operation e1.From(v2) returns (v1).
      ///
      /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
      /// (v1) and the head end (v2): (v1)->(v2). The operation e2.From(v1)
      /// returns (v2). The operation e2.From(v2) returns kNullId.
      ///
      /// \param[in] _from Source vertex.
      /// \return The other vertex of the edge reachable from the "_from"
      /// vertex or kNullId otherwise.
      public: virtual VertexId From(const VertexId &_from) const = 0;

      /// \brief Get the source end that can reach the destination end of
      /// an edge.
      ///
      /// E.g.: Let's assume that we have an undirected edge (e1) with ends
      /// (v1) and (v2): (v1)--(v2). The operation e1.To(v1) returns (v2).
      /// The operation e1.To(v2) returns (v1).
      ///
      /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
      /// (v1) and the head end (v2): (v1)->(v2). The operation e2.To(v1)
      /// returns kNullId. The operation e2.To(v2) returns v1.
      ///
      /// \param[in] _from Destination vertex.
      /// \return The other vertex of the edge that can reach "_to"
      /// vertex or kNullId otherwise.
      public: virtual VertexId To(const VertexId &_to) const = 0;

      /// \brief Get if the edge is valid. An edge is valid if its linked in a
      /// graph and its vertices are reachable.
      /// \return True when the edge is valid or false otherwise (invalid Id).
      public: bool Valid() const
      {
        return this->id != kNullId;
      }

      /// \brief Unique edge Id.
      private: EdgeId id = kNullId;
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
        for (auto const &vertex : this->vertices)
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
        auto vertices = _edge.Vertices();
        if (vertices.size() != 2u)
          return EdgeType::NullEdge;

        // Sanity check: Both vertices should exist.
        for (auto const &v : vertices)
        {
          auto itV = this->vertices.find(v);
          if (itV == this->vertices.end())
            return EdgeType::NullEdge;
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
        for (auto const &edge : this->edges)
        {
          res.emplace(std::make_pair(edge.first, std::cref(edge.second)));
        }

        return std::move(res);
      }

      /// \brief Get all vertices that are directly connected with one edge
      /// from a given vertex. E.g. j is adjacent from i if there is an
      /// edge (i->j).
      /// \param[in] _vertex The Id of the vertex to check adjacentsFrom.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> AdjacentsFrom(const VertexId &_vertex) const
      {
        VertexRef_M<V> res;

        auto vertexIt = this->adjList.find(_vertex);
        if (vertexIt == this->adjList.end())
          return res;

        for (auto const &edgeId : vertexIt->second)
        {
          auto edge = this->EdgeFromId(edgeId);
          auto neighborVertexId = edge.From(_vertex);
          if (neighborVertexId != kNullId)
          {
            auto neighborVertex = this->VertexFromId(neighborVertexId);
            res.emplace(
              std::make_pair(neighborVertexId, std::cref(neighborVertex)));
          }
        }

        return res;
      }

      /// \brief Get all neighbors vertices that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The vertex to check adjacents.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> AdjacentsFrom(const Vertex<V> &_vertex) const
      {
        return this->AdjacentsFrom(_vertex.Id());
      }

      /// \brief Get all vertices that are directly connected with one edge
      /// to a given vertex. E.g. i is adjacent to j if there is an
      /// edge (i->j).
      /// \param[in] _vertex The Id of the vertex to check adjacentsTo.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> AdjacentsTo(const VertexId &_vertex) const
      {
        auto incidentEdges = this->IncidentsTo(_vertex);

        VertexRef_M<V> res;
        for (auto incidentEdgeRef : incidentEdges)
        {
          auto incidentEdgeId = incidentEdgeRef.first;
          auto incidentEdge = this->EdgeFromId(incidentEdgeId);
          auto neighborVertexId = incidentEdge.To(_vertex);
          auto neighborVertex = this->VertexFromId(neighborVertexId);
          res.emplace(
              std::make_pair(neighborVertexId, std::cref(neighborVertex)));
        }

        return res;
      }

      /// \brief Get all vertices that are directly connected with one edge
      /// to a given vertex. E.g. i is adjacent to j if there is an
      /// edge (i->j).
      /// \param[in] _vertex The the vertex to check adjacentsTo.
      /// \return A map of vertices, where keys are Ids and values are
      /// references to the vertices.
      public: VertexRef_M<V> AdjacentsTo(const Vertex<V> &_vertex) const
      {
        return this->AdjacentsTo(_vertex.Id());
      }

      /// \brief Get the number of edges incidents to a vertex.
      /// \param[in] _vertex The vertex Id.
      /// \return The number of edges incidents to a vertex.
      public: size_t InDegree(const VertexId &_vertex) const
      {
        return this->IncidentsTo(_vertex).size();
      }

      /// \brief Get the number of edges incidents to a vertex.
      /// \param[in] _vertex The vertex.
      /// \return The number of edges incidents to a vertex.
      public: size_t InDegree(const Vertex<V> &_vertex) const
      {
        return this->IncidentsTo(this->VertexFromId(_vertex.Id())).size();
      }

      /// \brief Get the number of edges incidents from a vertex.
      /// \param[in] _vertex The vertex Id.
      /// \return The number of edges incidents from a vertex.
      public: size_t OutDegree(const VertexId &_vertex) const
      {
        return this->AdjacentsFrom(_vertex).size();
      }

      /// \brief Get the number of edges incidents from a vertex.
      /// \param[in] _vertex The vertex.
      /// \return The number of edges incidents from a vertex.
      public: size_t OutDegree(const Vertex<V> &_vertex) const
      {
        return this->AdjacentsFrom(this->VertexFromId(_vertex.Id())).size();
      }

      /// \brief Get the set of outgoing edges from a given vertex.
      /// \param[in] _vertex Id of the vertex.
      /// \return A map of edges, where keys are Ids and values are
      /// references to the edges.
      public: EdgeRef_M<EdgeType> IncidentsFrom(const VertexId &_vertex) const
      {
        EdgeRef_M<EdgeType> res;

        auto adjIt = this->adjList.find(_vertex);
        if (adjIt == this->adjList.end())
          return res;

        auto edgeIds = adjIt->second;
        for (auto const &edgeId : edgeIds)
        {
          auto edge = this->EdgeFromId(edgeId);
          if (edge.From(_vertex) != kNullId)
            res.emplace(std::make_pair(edge.Id(), std::cref(edge)));
        }

        return std::move(res);
      }

      /// \brief Get the set of outgoing edges from a given vertex.
      /// \param[in] _vertex The vertex.
      /// \return A map of edges, where keys are Ids and values are
      /// references to the edges.
      public: EdgeRef_M<EdgeType> IncidentsFrom(const Vertex<V> &_vertex) const
      {
        return this->IncidentsFrom(_vertex.Id());
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Id of the vertex.
      /// \return A map of edges, where keys are Ids and values are
      /// references to the edges.
      public: EdgeRef_M<EdgeType> IncidentsTo(const VertexId &_vertex) const
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
      public: EdgeRef_M<EdgeType> IncidentsTo(const Vertex<V> &_vertex) const
      {
        return this->IncidentsTo(_vertex.Id());
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
        auto incidents = this->IncidentsTo(_vertex);
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
        size_t numVertices = this->names.count(_name);
        for (auto i = 0; i < numVertices; ++i)
        {
          auto iter = this->names.find(_name);
          this->RemoveVertex(iter->second);
        }

        return numVertices;
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
