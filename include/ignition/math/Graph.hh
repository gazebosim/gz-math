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

#include <algorithm>
#include <cassert>
// int64_t
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace ignition
{
  namespace math
  {
    using VertexId = int64_t;
    using EdgeId = int64_t;

    /// \brief A vertex of a graph. It stores user information and keeps
    /// an internal unique Id.
    template<typename V>
    class Vertex
    {
      /// \brief Constructor.
      /// \param[in] _name Non-unique vertex name.
      /// \param[in] _id Unique id.
      /// \param[in] _data User information.
      public: Vertex(const std::string &_name,
                     const VertexId _id = -1, const V &_data = V())

        : data(_data),
          name(_name),
          id(_id)
      {
      }

      /// \brief Retrieve the user information.
      /// \return A mutable reference to the user information.
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

      /// \brief User information.
      private: V data;

      /// \brief Unique vertex Id.
      private: VertexId id = -1;

      /// \brief Non-unique vertex name.
      private: std::string name = "";
    };

    /// \def VertexPtr
    /// \brief Shared pointer to a vertex.
    template<typename V>
    using VertexPtr = std::shared_ptr<Vertex<V>>;


    /// \def VertexPtr_V
    /// \brief Vector of shared pointers to a vertex.
    template<typename V>
    using VertexPtr_V = std::vector<VertexPtr<V>>;

    /// \def VertexPtr_S
    /// \brief Set of shared pointers to a vertex.
    template<typename V>
    using VertexPtr_S = std::set<VertexPtr<V>>;

    /// \brief Generic edge class. An edge has two ends and some constraint
    /// between them. For example, a directed edge only allow traversing the
    /// edge in one direction.
    template<typename E>
    class Edge
    {
      public: Edge(const VertexId _tail, const VertexId _head,
                  const E &_data = E())
              : tail (_tail), head(_head), data(_data)
              {
              }

      /// \brief Get the set of ends of the edge.
      /// \return The set of ends of the edge.
      // public: virtual std::set<VertexId> Vertexes() const = 0;

      /// \brief Get the destination end that is reachable from a source end of
      /// an edge.
      ///
      /// E.g.: Let's assume that we have an undirected edge (e1) with ends
      /// (v1) and (v2). The operation e1.To(v1) returns (v2).
      /// The operation e1.To(v2) returns (v1).
      ///
      /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
      /// (v1) and the head end (v2). The operation e2.To(v1) returns (v2).
      /// The operation e2.To(v2) returns nullptr.
      ///
      /// \param[in] _from Source vertex.
      /// \return The other vertex of the edge reachable from the "_from"
      /// vertex or nullptr otherwise.
      public: VertexId To(const int64_t &_from) const
      {
        if (_from != this->Tail())
          return -1;

        return this->Head();
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      /// \brief Get a shared pointer to the head's vertex in this edge.
      /// \return A shared pointer to the head's vertex in this edge.
      /// \sa Tail()
      public: VertexId Head() const
      {
        if (!this->Valid())
          return -1;

        return this->head;
      }

      // Documentation inherited.
      public: std::set<VertexId> Vertexes() const
      {
        if (!this->Valid())
          return {-1, -1};

        return {this->tail, this->head};
      }

      /// \brief Get a shared pointer to the tail's vertex in this edge.
      /// \return A shared pointer to the tail's vertex in this edge.
      /// \sa Head()
      public: VertexId Tail() const
      {
        if (!this->Valid())
          return -1;

        return this->tail;
      }

      /// \brief Get if the edge is valid. An edge is valid if its linked in a
      /// graph and its vertexes are reachable.
      /// \return True when the edge is valid.
      /// \sa SetValid
      public: bool Valid() const
      {
        return this->valid;
      }

      /// \brief Set an edge valid or invalid.
      /// \param[in] _newValue Validity.
      /// \sa Valid
      public: void SetValid(const bool _newValue)
      {
        this->valid = _newValue;
      }

      /// \brief True when the edge is connected in a graph or false otherwise.
      /// This member variable exists to prevent the following situation:
      /// Imagine that you have a shared pointer pointing to a valid edge
      /// connected in a graph. Now, you use the Graph API and remove the
      /// edge. This operation will disconnect the edge from the graph but the
      /// edge won't be deallocated because you will keep a shared pointer.
      /// Having a shared pointer to the edge will allow you to traverse the
      /// edge and go to any of the vertexes of the graph and modify it.
      /// Once an edge is removed, this flag is set to false and this will
      /// prevent you from traversing the edge and reach the vertexes.
      private: bool valid = false;

      /// \brief Shared pointer to the tail's vertex.
      protected: int64_t tail;

      /// \brief Shared pointer to the head's vertex.
      protected: int64_t head;

      /// \brief User data.
      protected: E data;
    };

    /// \def EdgePtr
    /// \brief Shared pointer to an edge.
    template<typename EdgeType>
    using EdgePtr = std::shared_ptr<EdgeType>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename EdgeType>
    using EdgePtr_S = std::set<EdgePtr<EdgeType>>;

    /// \def AdjList
    /// \brief A map where the keys are pointers to all vertexes. For each
    /// vertex (v), the map value contains a set of pointers to edges. Each of
    /// these edges (e) represents that there is a path from (v) to another
    /// vertex via (e).
    template<typename V, typename EdgeType>
    using AdjList = std::map<VertexPtr<V>, EdgePtr_S<EdgeType>>;

    /// \brief A generic graph class.
    /// Both vertexes and edges can store user information. A vertex could be
    /// created passing a custom Id if needed, otherwise it will be choosen
    /// internally. The vertexes also have a name that could be reused among
    /// other vertexes if needed. This class supports the use of different edge
    /// types (e.g. directed or undirected edges).
    template<typename V, typename E, typename EdgeType>
    class Graph
    {
      public: static Vertex<V> NullVertex;

      /// \brief Default constructor.
      public: Graph() = default;

      public: EdgeId AddEdge(const VertexId _tail, const VertexId _head,
                             const E &_data)
      {
        if (this->vertexes.find(_tail) == this->vertexes.end() ||
            this->vertexes.find(_head) == this->vertexes.end())
        {
          // Error, vertexes don't exist;
          return -1;
        }

        auto id = this->NextEdgeId();
        this->edges.insert(std::make_pair(id,
              Edge<E>(_tail, _head, _data)));
        // this->edges[id].SetValid(true);

        this->connections[_tail].insert(_head);
        this->connections[_head].insert(_tail);

        return id;
      }

      /// \brief Add a new vertex to the graph.
      /// \param[in] _data Data to be stored in the vertex.
      /// \param[in] _name Name of the vertex. It doesn't have to be unique.
      /// \param[in] _id Optional Id to be used for this vertex.
      public: VertexId AddVertex(const std::string &_name,
                                 const VertexId _id = -1, const V &_data = V())
      {
        auto id = _id;
        // The user didn't provide an Id, we generate it.
        if (id == -1)
          id = this->NextId();
        // The user provided an Id but already exists.
        else if (this->vertexes.find(id) != this->vertexes.end())
          return -1;

        // Create the vertex.
        this->vertexes.insert(std::make_pair(id, Vertex<V>(_name, id, _data)));

        // Update the map of names.
        this->names[_name] = id;

        return id;
      }

      /// \brief The collection of all vertexes in the graph.
      /// \return A set of shared pointers to all vertexes.
      public: const std::map<int64_t, Vertex<V>> &Vertexes() const
      {
        return this->vertexes;
      }

      /// \brief The collection of all vertexes in the graph with name == _name.
      /// \return A vector of shared pointers to all vertexes with name == _name
      /*public: VertexPtr_S<V> Vertexes(const std::string &_name) const
      {
        VertexPtr_S<V> res;
        for (auto const &nodeAdjList : this->edges)
        {
          if (nodeAdjList.first->Name() == _name)
            res.insert(nodeAdjList.first);
        }

        return res;
      }*/


      /// \brief The collection of all edges in the graph.
      /// \return The set of all edges in the graph.
      public: std::map<EdgeId, Edge<E>> &Edges()
      {
        return this->edges;
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The pointer to the vertex to check adjacents.
      /// \return A set of vertexes that are adjacents and directly connected
      /// with an edge.
      public: std::set<VertexId> Adjacents(const int64_t &_vertex) const
      {
        auto vertexIt = this->connections.find(_vertex);
        if (vertexIt == this->connections.end())
          return {};

        std::set<VertexId> res;
        for (auto const &edge : vertexIt->second)
          res.insert(this->connections[edge].To(_vertex));

        return res;
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Pointer to the vertex.
      /// \return The set of incoming edges to a given vertex.
      public: std::set<EdgeId> Incidents(const int64_t &_vertex) const
      {
        std::set<EdgeId> res;
        auto vertexIt = this->connections.find(_vertex);
        if (vertexIt == this->connections.end())
          return {};

        for (auto const &nodeAdjList : this->connections)
        {
          auto edgs = nodeAdjList.second;
          for (auto const &e : edgs)
          {
            if (this->edges[e].To(
                  this->vertexes[nodeAdjList.first]) == _vertex)
            {
              res.insert(e);
            }
          }
        }

        return res;
      }

      /// \brief Whether the graph is empty.
      /// \return True when there are no vertexes in the graph or
      /// false otherwise.
      public: bool Empty() const
      {
        return this->vertexs.empty() && this->edges.empty();
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex Pointer to the vertex to be removed.
      /*public: void RemoveVertex(VertexPtr<V> &_vertex)
      {
        if (!_vertex)
          return;

        // Save the Id.
        auto id = _vertex->Id();

        // Remove incident edges.
        auto incidents = this->Incidents(_vertex);
        for (auto e : incidents)
          this->RemoveEdge(e);

        // Remove the vertex and all outcoming edges.
        auto itPair = std::find_if(this->edges.begin(), this->edges.end(),
          [&_vertex](std::pair<int64_t, EdgePtr_S<EdgeType>> _pair)
          {
             return _pair.first == _vertex->Id();
          });
        if (itPair == this->edges.end())
          return;

        this->edges.erase(itPair);

        // Remove also the id from the map of Ids.
        this->ids.erase(id);

        // Remove also the vertex from the map of names.
        std::string name = _vertex->Name();
        assert(this->names.find(name) != this->names.end());

        auto &v = this->names.at(name);
        v.erase(std::remove(v.begin(), v.end(), _vertex), v.end());
        if (v.empty())
          this->names.erase(name);

        this->vertexes.erase(_vertex->Id());
      }

      /// \brief Remove all vertexes with name == _name.
      /// \param[in] _name Name of the vertexes to be removed.
      public: void RemoveVertexes(const std::string &_name)
      {
        auto iter = this->names.find(_name);
        if (iter == this->names.end())
          return;

        auto &v = iter->second;
        while (!v.empty())
          this->RemoveVertex(v.front());
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertexes from the edge.
      /// \param[in] _edge Pointer to the edge to be removed.
      public: void RemoveEdge(EdgePtr<EdgeType> &_edge)
      {
        if (!_edge)
          return;

        auto edgeVertexes = _edge->Vertexes();
        if (edgeVertexes.size() != 2u)
          return;

        // Sanity check: Both vertexes should exist.
        for (auto const &v : edgeVertexes)
        {
          auto itV = this->edges.find(v->Id());
          if (itV == this->edges.end())
            return;
        }

        // Unlink the edge.
        for (auto const &v : edgeVertexes)
        {
          if (_edge->To(v) != nullptr)
          {
            auto vertex = this->edges.find(v->Id());
            assert(vertex != this->edges.end());
            vertex->second.erase(_edge);
          }
        }

        // Mark the edge as invalid. This will prevent to reach any vertexes if
        // there are any shared pointers keeping the edge alive.
        _edge->SetValid(false);
      }*/

      /// \brief Get a pointer to a vertex using its Id.
      /// \param[in] _id The ID of the vertex.
      /// \return A shared pointer to the vertex with Id = _id or nullptr if
      /// not found.
      public: Vertex<V> &VertexById(const int64_t _id)
      {
        auto iter = this->vertexes.find(_id);
        if (iter == this->vertexes.end())
          return NullVertex;

        return iter->second;
      }

      /// \brief Get an available Id to be assigned to a new vertex.
      /// \return The next available Id.
      private: VertexId NextId()
      {
        while (this->vertexes.find(this->nextId) != this->vertexes.end())
          ++this->nextId;

        return this->nextId;
      }

      private: EdgeId NextEdgeId()
      {
        while (this->edges.find(this->nextEdgeId) != this->edges.end())
          ++this->nextEdgeId;

        return this->nextEdgeId;
      }

      /// The directed graph is represented using an adjacency list.
      protected: std::map<VertexId, Vertex<V>> vertexes;
      protected: std::map<EdgeId, Edge<E>> edges;

      //protected: std::map<VertexId, EdgePtr_S<EdgeType>> edges;
      protected: std::map<VertexId, std::set<EdgeId>> connections;

      /// \brief Associatation between names and vertexes curently used.
      protected: std::map<std::string, VertexId> names;

      /// \brief The next vertex Id to be assigned to a new vertex.
      private: VertexId nextId = 0;

      /// \brief The next vertex Id to be assigned to a new vertex.
      private: EdgeId nextEdgeId = 0;
    };

    template<typename V, typename E, typename EdgeType>
    Vertex<V> Graph<V, E, EdgeType>::NullVertex("__null__", -1);
  }
}
#endif
