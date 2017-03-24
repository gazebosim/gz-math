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
#include <cstdint>
#include <map>
#include <memory>
#include <iostream>
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

    // Forward declaration.
    template<typename V, typename E, typename EdgeType>
    class Graph;

    /// \brief Generic edge class.
    template<typename V>
    class Edge
    {
      /// \brief
      public: virtual VertexPtr_S<V> Vertexes() const = 0;

      /// \brief ToDo.
      public: virtual VertexPtr<V> To(const VertexPtr<V> _from) const = 0;

      /// \brief ToDo.
      public: bool Valid() const
      {
        return this->valid;
      }

      /// \brief ToDo.
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

     /// \def
    /// \brief Shared pointer to an edge.
    template<typename EdgeType>
    using GenericEdgePtr = std::shared_ptr<EdgeType>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename EdgeType>
    using GenericEdgePtr_S = std::set<GenericEdgePtr<EdgeType>>;

    template<typename V, typename EdgeType>
    using GenericAdjList = std::map<VertexPtr<V>, GenericEdgePtr_S<EdgeType>>;

    template<typename V, typename EdgeType>
    using GenericNodeIt = typename GenericAdjList<V, EdgeType>::const_iterator;

    /// ToDo.
    template<typename EdgeType>
    using GenericEdgeIt = typename GenericEdgePtr_S<EdgeType>::const_iterator;

    // Adjacency iterator.
    template<typename V, typename EdgeType>
    class GenericAdjIt
    {
      /// \brief ToDo.
      public: GenericAdjIt(GenericNodeIt<V, EdgeType> n):
        edgeBegin(n->second.begin()),
        edgeEnd(n->second.end()),
        node(n),
        adj(n->second.begin())
      {
      }

      /// \brief ToDo.
      public: bool Valid() const
      {
        return this->adj != this->edgeEnd;
      }

      /// \brief ToDo.
      public: V Value() const
      {
        return this->node->first->Data();
      }

      /// \brief ToDo.
      public: GenericEdgeIt<EdgeType> CurAdj() const
      {
        return this->adj;
      }

      /// \brief ToDo.
      public: GenericAdjIt &operator++()
      {
        if (this->adj != this->edgeEnd)
          ++this->adj;
        return *this;
      }

      /// ToDo.
      private: GenericEdgeIt<EdgeType> const edgeBegin;
      /// ToDo.
      private: GenericEdgeIt<EdgeType> const edgeEnd;
      /// ToDo.
      private: GenericNodeIt<V, EdgeType> node;
      /// ToDo.
      private: GenericEdgeIt<EdgeType> adj;
    };

    // Graph edge iterator.
    template<typename V, typename EdgeType>
    class GenericGraphEdgeIt
    {
      /// \brief ToDo.
      public: GenericGraphEdgeIt(const GenericNodeIt<V, EdgeType> _b,
                                 const GenericNodeIt<V, EdgeType> _e):
        end(_e),
        curVertex(_b),
        curEdge(_b->second.begin()),
        edgeEnd(_b->second.end())
      {
        if (this->curEdge == this->edgeEnd)
          this->FindNextEdge();
      }

      /// \brief ToDo.
      public: bool Valid() const
      {
        return this->curVertex != this->end;
      }

      /// \brief ToDo.
      public: GenericNodeIt<V, EdgeType> CurVertex() const
      {
        return this->curVertex;
      }

      /// \brief ToDo.
      public: GenericEdgeIt<EdgeType> CurEdge() const
      {
        return this->curEdge;
      }

      /// \brief ToDo.
      public: GenericGraphEdgeIt &operator++()
      {
        this->FindNextEdge();
        return *this;
      }

      /// \brief ToDo.
      private: void FindNextEdge()
      {
        if (this->curEdge != this->edgeEnd)
          ++this->curEdge;

        while ((this->curEdge   == this->edgeEnd) &&
               (this->curVertex != this->end))
        {
          ++this->curVertex;
          this->curEdge = this->curVertex->second.begin();
          this->edgeEnd = this->curVertex->second.end();
        }
      }

      /// ToDo.
      private: GenericNodeIt<V, EdgeType> const end;
      /// ToDo.
      private: GenericNodeIt<V, EdgeType> curVertex;
      /// ToDo.
      private: GenericEdgeIt<EdgeType> curEdge;
      /// ToDo.
      private: GenericEdgeIt<EdgeType> edgeEnd;
    };

    /// \brief A generic directed graph class.
    /// Both vertexes and edges can store user information. A vertex could be
    /// created passing a custom Id if needed, otherwise it will be choosen
    /// internally. The vertexes also have a name that could be reused among
    /// other vertexes if needed.
    template<typename V, typename E, typename EdgeType>
    class Graph
    {
      /// \brief Default constructor.
      public: Graph() = default;

      /// \brief ToDo.
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
        this->data[v] = GenericEdgePtr_S<EdgeType>();

        // Update the map of Ids.
        this->ids[id] = v;
        // Update the map of names.
        this->names[_name].push_back(v);

        return v;
      }

      /// \brief ToDo.
      /// \return A vector of shared pointers to all vertexes
      public: VertexPtr_S<V> Vertexes() const
      {
        VertexPtr_S<V> res;
        for (auto nodeAdjList : this->data)
          res.insert(nodeAdjList.first);

        return res;
      }

      /// \brief ToDo.
      /// \return A vector of shared pointers to all vertexes
      public: VertexPtr_S<V> Vertexes(const std::string &_name) const
      {
        VertexPtr_S<V> res;
        for (auto nodeAdjList : this->data)
        {
          if (nodeAdjList.first->Name() == _name)
            res.insert(nodeAdjList.first);
        }

        return res;
      }

      /// \brief ToDo.
      public: bool LinkEdge(GenericEdgePtr<EdgeType> _edge)
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

        // Mark the edge as valid.
        _edge->SetValid(true);

        return true;
      }

      /// \brief ToDo.
      public: GenericEdgePtr_S<EdgeType> Edges() const
      {
        GenericGraphEdgeIt<V, EdgeType> graphEdgeIt(this->begin(), this->end());

        GenericEdgePtr_S<EdgeType> res;
        for (; graphEdgeIt.Valid(); ++graphEdgeIt)
          res.insert(*graphEdgeIt.CurEdge());

        return res;
      }

      /// \brief ToDo.
      public: VertexPtr_S<V> Adjacents(const VertexPtr<V> _vertex)
      {
        // Get a vertex iterator from the vertex Id.
        typename GenericAdjList<V, EdgeType>::iterator vIt =
          this->Find(_vertex);
        GenericAdjIt<V, EdgeType> adjIt(vIt);

        VertexPtr_S<V> res;
        for (; adjIt.Valid(); ++adjIt)
          res.insert((*adjIt.CurAdj())->To(_vertex));

        return res;
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _id The vertex ID to check adjacent vertexes.
      /// \return A vector of vertexes that are adjacents and directly connected
      /// with an edge.
      public: VertexPtr_S<V> Adjacents(const int64_t _id)
      {
        return this->Adjacents(this->VertexById(_id));
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Pointer to the vertex.
      /// \return The set of incoming edges to a given vertex.
      public: GenericEdgePtr_S<EdgeType> Incidents(const VertexPtr<V> _vertex)
      {
        GenericEdgePtr_S<EdgeType> res;

        for (auto nodeAdjList : this->data)
        {
          auto edges = nodeAdjList.second;
          for (auto e : edges)
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
      public: GenericEdgePtr_S<EdgeType> Incidents(const int64_t _id)
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
      public: void RemoveVertex(VertexPtr<V> &_vertex)
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
        auto itPair = std::find_if(this->data.begin(), this->data.end(),
          [&_vertex](std::pair<VertexPtr<V>, GenericEdgePtr_S<EdgeType>> _pair)
          {
             return _pair.first == _vertex;
          });
        if (itPair == this->data.end())
          return;

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
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _id ID of the vertex to be removed.
      public: void RemoveVertex(const int64_t _id)
      {
        auto vPtr = this->VertexById(_id);
        if (vPtr)
          this->RemoveVertex(vPtr);
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
      /// won't be possible to reach any of the vertexes from the edge. Any
      /// call to Tail() or Head() will return nullptr.
      /// \param[in] _edge Pointer to the edge to be removed.
      public: void RemoveEdge(GenericEdgePtr<EdgeType> &_edge)
      {
        if (!_edge)
          return;

        auto vertexes = _edge->Vertexes();
        if (vertexes.size() != 2u)
          return;

        // Sanity check: Both vertexes should exist.
        for (auto const &v : vertexes)
        {
          auto itV = this->data.find(v);
          if (itV == this->data.end())
            return;
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
      }

      /// \brief Get a pointer to a vertex using its Id.
      /// \param[in] _id The ID of the vertex.
      /// \return A shared pointer to the vertex with Id = _id or nullptr if
      /// not found.
      public: VertexPtr<V> VertexById(const int64_t _id)
      {
        auto iter = this->ids.find(_id);
        if (iter == this->ids.end())
          return nullptr;

        return iter->second;
      }

      /// \brief Get an iterator pointing to the first vertex in the graph.
      public: GenericNodeIt<V, EdgeType> begin() const
      {
        return this->data.begin();
      }

      /// \brief Get an iterator pointing to the past-the-end vertex in the
      /// graph.
      public: GenericNodeIt<V, EdgeType> end() const
      {
        return this->data.end();
      }

      /// \brief ToDo.
      public: typename GenericAdjList<V, EdgeType>::iterator Find(
                                                    const VertexPtr<V> &_vertex)
      {
        return this->data.find(_vertex);
      }

      /// \brief ToDo.
      public: typename GenericAdjList<V, EdgeType>::iterator Find(
                                                        const int64_t _vertexId)
      {
        return this->Find(this->VertexById(_vertexId));
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
      protected: GenericAdjList<V, EdgeType> data;

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
