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
    /// \brief A vertex of a graph. It stores user information and keeps
    /// an internal unique Id.
    template<typename V>
    class Vertex
    {
      /// \brief Constructor.
      /// \param[in] _data User information.
      /// \param[in] _name Non-unique vertex name.
      /// \param[in] _id Unique id.
      public: Vertex(const V &_data,
                     const std::string &_name,
                     const int64_t _id)
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
      public: int64_t Id() const
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
      private: int64_t id;

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
    template<typename V>
    class Edge
    {
      /// \brief Get the set of ends of the edge.
      /// \return The set of ends of the edge.
      public: virtual VertexPtr_S<V> Vertexes() const = 0;

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
      public: virtual VertexPtr<V> To(const VertexPtr<V> &_from) const = 0;

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
      /// \brief Default constructor.
      public: Graph() = default;

      /// \brief Add a new vertex to the graph.
      /// \param[in] _data Data to be stored in the vertex.
      /// \param[in] _name Name of the vertex. It doesn't have to be unique.
      /// \param[in] _id Optional Id to be used for this vertex.
      public: VertexPtr<V> AddVertex(const V &_data,
                                     const std::string &_name,
                                     const int64_t _id = -1)
      {
        auto id = _id;
        // The user didn't provide an Id, we generate it.
        if (id == -1)
          id = this->NextId();
        // The user provided an Id but already exists.
        else if (this->ids.find(id) != this->ids.end())
          return nullptr;

        // Create the vertex.
        auto v = std::make_shared<Vertex<V>>(_data, _name, id);

        // Link the vertex with an empty list of edges.
        this->data[v] = EdgePtr_S<EdgeType>();

        // Update the map of Ids.
        this->ids[id] = v;
        // Update the map of names.
        this->names[_name].push_back(v);

        return v;
      }

      /// \brief The collection of all vertexes in the graph.
      /// \return A set of shared pointers to all vertexes.
      public: VertexPtr_S<V> Vertexes() const
      {
        VertexPtr_S<V> res;
        for (auto const &nodeAdjList : this->data)
          res.insert(nodeAdjList.first);

        return res;
      }

      /// \brief The collection of all vertexes in the graph with name == _name.
      /// \return A vector of shared pointers to all vertexes with name == _name
      public: VertexPtr_S<V> Vertexes(const std::string &_name) const
      {
        VertexPtr_S<V> res;
        for (auto const &nodeAdjList : this->data)
        {
          if (nodeAdjList.first->Name() == _name)
            res.insert(nodeAdjList.first);
        }

        return res;
      }

      /// \brief Links an edge to the graph.
      /// \param[in] _edge A new edge.
      /// \return True when the operation succeed or false otherwise.
      public: bool LinkEdge(EdgePtr<EdgeType> &_edge)
      {
        _edge->SetValid(true);

        auto vertexes = _edge->Vertexes();
        if (vertexes.size() != 2u)
        {
          _edge->SetValid(false);
          return false;
        }

        // Sanity check: Both vertexes should exist.
        for (auto const &v : vertexes)
        {
          auto itV = this->data.find(v);
          if (itV == this->data.end())
          {
            _edge->SetValid(false);
            return false;
          }
        }

        // Link the new edge.
        for (auto const &v : vertexes)
        {
          if (_edge->To(v) != nullptr)
          {
            auto vertex = this->data.find(v);
            assert(vertex != this->data.end());
            vertex->second.insert(_edge);
          }
        }

        return true;
      }

      /// \brief The collection of all edges in the graph.
      /// \return The set of all edges in the graph.
      public: EdgePtr_S<EdgeType> Edges() const
      {
        EdgePtr_S<EdgeType> res;
        for (auto const &vertex : this->data)
          res.insert(vertex.second.begin(), vertex.second.end());

        return res;
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The pointer to the vertex to check adjacents.
      /// \return A set of vertexes that are adjacents and directly connected
      /// with an edge.
      public: VertexPtr_S<V> Adjacents(const VertexPtr<V> &_vertex) const
      {
        auto vertexIt = this->data.find(_vertex);
        if (vertexIt == this->data.end())
          return {};

        VertexPtr_S<V> res;
        for (auto const &edge : vertexIt->second)
          res.insert(edge->To(_vertex));

        return res;
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _id The vertex ID to check adjacent vertexes.
      /// \return A set of vertexes that are adjacents and directly connected
      /// with an edge.
      public: VertexPtr_S<V> Adjacents(const int64_t _id) const
      {
        return this->Adjacents(this->VertexById(_id));
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Pointer to the vertex.
      /// \return The set of incoming edges to a given vertex.
      public: EdgePtr_S<EdgeType> Incidents(const VertexPtr<V> _vertex) const
      {
        EdgePtr_S<EdgeType> res;
        auto vertexIt = this->data.find(_vertex);
        if (vertexIt == this->data.end())
          return {};

        for (auto const &nodeAdjList : this->data)
        {
          auto edges = nodeAdjList.second;
          for (auto const &e : edges)
          {
            if (e->To(nodeAdjList.first) == _vertex)
              res.insert(e);
          }
        }

        return res;
      }

      /// \brief Get the set of incoming edges to a given node.
      /// \param[in] _id The ID of the vertex.
      /// \return The set of incoming edges to a given node.
      public: EdgePtr_S<EdgeType> Incidents(const int64_t _id) const
      {
        return this->Incidents(this->VertexById(_id));
      }

      /// \brief Whether the graph is empty.
      /// \return True when there are no vertexes in the graph or
      /// false otherwise.
      public: bool Empty() const
      {
        return this->data.empty();
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex Pointer to the vertex to be removed.
      /// \return True when the vertex was removed or false otherwise.
      public: bool RemoveVertex(VertexPtr<V> &_vertex)
      {
        if (!_vertex)
          return false;

        // Save the Id.
        auto id = _vertex->Id();

        // Remove incident edges.
        auto incidents = this->Incidents(_vertex);
        for (auto e : incidents)
          this->RemoveEdge(e);

        // Remove the vertex and all outcoming edges.
        auto itPair = std::find_if(this->data.begin(), this->data.end(),
          [&_vertex](std::pair<VertexPtr<V>, EdgePtr_S<EdgeType>> _pair)
          {
             return _pair.first == _vertex;
          });
        if (itPair == this->data.end())
          return false;

        this->data.erase(itPair);

        // Remove also the id from the map of Ids.
        this->ids.erase(id);

        // Remove also the vertex from the map of names.
        std::string name = _vertex->Name();
        assert(this->names.find(name) != this->names.end());

        auto &v = this->names.at(name);
        v.erase(std::remove(v.begin(), v.end(), _vertex), v.end());
        if (v.empty())
          this->names.erase(name);

        return true;
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _id ID of the vertex to be removed.
      /// \return True when the vertex was removed or false otherwise.
      public: bool RemoveVertex(const int64_t _id)
      {
        auto vPtr = this->VertexById(_id);
        return this->RemoveVertex(vPtr);
      }

      /// \brief Remove all vertexes with name == _name.
      /// \param[in] _name Name of the vertexes to be removed.
      /// \return True when at least one vertex was removed.
      public: bool RemoveVertexes(const std::string &_name)
      {
        auto iter = this->names.find(_name);
        if (iter == this->names.end())
          return false;

        auto &v = iter->second;
        while (!v.empty())
          this->RemoveVertex(v.front());

        return true;
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertexes from the edge.
      /// \param[in] _edge Pointer to the edge to be removed.
      /// \return True when the edge was removed.
      public: bool RemoveEdge(EdgePtr<EdgeType> &_edge)
      {
        if (!_edge)
          return false;

        auto vertexes = _edge->Vertexes();
        if (vertexes.size() != 2u)
          return false;

        // Sanity check: Both vertexes should exist.
        for (auto const &v : vertexes)
        {
          auto itV = this->data.find(v);
          if (itV == this->data.end())
            return false;
        }

        // Unlink the edge.
        for (auto const &v : vertexes)
        {
          if (_edge->To(v) != nullptr)
          {
            auto vertex = this->data.find(v);
            assert(vertex != this->data.end());
            vertex->second.erase(_edge);
          }
        }

        // Mark the edge as invalid. This will prevent to reach any vertexes if
        // there are any shared pointers keeping the edge alive.
        _edge->SetValid(false);

        return true;
      }

      /// \brief Get a pointer to a vertex using its Id.
      /// \param[in] _id The ID of the vertex.
      /// \return A shared pointer to the vertex with Id = _id or nullptr if
      /// not found.
      public: VertexPtr<V> VertexById(const int64_t _id) const
      {
        auto iter = this->ids.find(_id);
        if (iter == this->ids.end())
          return nullptr;

        return iter->second;
      }

      /// \brief Get an available Id to be assigned to a new vertex.
      /// \return The next available Id.
      private: int64_t NextId()
      {
        while (this->ids.find(this->nextId) != this->ids.end())
          ++this->nextId;

        return this->nextId;
      }

      /// The directed graph is represented using an adjacency list.
      protected: AdjList<V, EdgeType> data;

      /// \brief List of ids curently used.
      protected: std::map<int64_t, VertexPtr<V>> ids;

      /// \brief Associatation between names and vertexes curently used.
      protected: std::map<std::string, VertexPtr_V<V>> names;

      /// \brief The next vertex Id to be assigned to a new vertex.
      private: int64_t nextId = 0;
    };
  }
}
#endif
