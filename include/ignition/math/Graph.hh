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

    /// \brief Used in the Graph constructor as a uniform initialization.
    template<typename E>
    struct EdgeInitializer
    {
      /// \brief ID of the tail's vertex.
      public: int64_t tailId;

      /// \brief ID of the head's vertex.
      public: int64_t headId;

      /// \brief User data.
      public: E data;
    };

    /// \def VertexPtr
    /// \brief Shared pointer to a vertex.
    template<typename V>
    using VertexPtr = std::shared_ptr<Vertex<V>>;

    /// \def VertexPtr_V
    /// \brief Vector of shared pointers to a vertex.
    template<typename V>
    using VertexPtr_V = std::vector<VertexPtr<V>>;

    // Forward declarations.
    template <typename V, typename E>
    class DirectedGraph;

    /// \brief An edge represents a connection between two vertexes.
    template<typename V, typename E>
    class Edge
    {
      /// \brief Constructor.
      /// \param[in] _tail Shared pointer to the tail vertex.
      /// \param[in] _head Shared pointer to the head vertex.
      /// \param[in] _data User data to be stored in the edge.
      public: Edge(const VertexPtr<V> _tail,
                   const VertexPtr<V> _head,
                   const E &_data)
        : tail(_tail),
          head(_head),
          data(_data)
      {
      }

      /// \brief Get a shared pointer to the tail's vertex in this edge.
      /// \return A shared pointer to the tail's vertex in this edge.
      public: VertexPtr<V> Tail() const
      {
        if (!this->valid)
          return nullptr;

        return this->tail;
      }

      /// \brief Get a shared pointer to the head's vertex in this edge.
      /// \return A shared pointer to the head's vertex in this edge.
      public: VertexPtr<V> Head() const
      {
        if (!this->valid)
          return nullptr;

        return this->head;
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      /// \brief Shared pointer to the tail's vertex.
      private: VertexPtr<V> tail;

      /// \brief Shared pointer to the head's vertex.
      private: VertexPtr<V> head;

      /// \brief User data.
      private: E data;

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

      /// The Graph class needs to modify 'valid' during edge removal.
      friend class DirectedGraph<V, E>;
    };

    /// \def EdgePtr
    /// \brief Shared pointer to an edge.
    template<typename V, typename E>
    using EdgePtr = std::shared_ptr<Edge<V, E>>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename V, typename E>
    using EdgePtr_S = std::set<EdgePtr<V, E>>;

    template<typename V, typename E>
    using AdjList = std::map<VertexPtr<V>, EdgePtr_S<V, E>>;

    /// ToDo.
    template<typename V, typename E>
    using NodeIt = typename AdjList<V, E>::const_iterator;

    /// ToDo.
    template<typename V, typename E>
    using EdgeIt = typename EdgePtr_S<V, E>::const_iterator;

    // Adjacency iterator.
    template<typename V, typename E>
    class AdjIt
    {
      /// \brief ToDo.
      public: AdjIt(NodeIt<V, E> n):
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
      public: EdgeIt<V, E> CurAdj() const
      {
        return this->adj;
      }

      /// \brief ToDo.
      public: AdjIt &operator++()
      {
        if (this->adj != this->edgeEnd)
          ++this->adj;
        return *this;
      }

      /// ToDo.
      private: EdgeIt<V, E> const edgeBegin;
      /// ToDo.
      private: EdgeIt<V, E> const edgeEnd;
      /// ToDo.
      private: NodeIt<V, E> node;
      /// ToDo.
      private: EdgeIt<V, E> adj;
    };

    // Graph edge iterator.
    template<typename V, typename E>
    class GraphEdgeIt
    {
      /// \brief ToDo.
      public: GraphEdgeIt(const NodeIt<V, E> _b,
                          const NodeIt<V, E> _e):
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
      public: NodeIt<V, E> CurVertex() const
      {
        return this->curVertex;
      }

      /// \brief ToDo.
      public: EdgeIt<V, E> CurEdge() const
      {
        return this->curEdge;
      }

      /// \brief ToDo.
      public: GraphEdgeIt &operator++()
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
      private: NodeIt<V, E> const end;
      /// ToDo.
      private: NodeIt<V, E> curVertex;
      /// ToDo.
      private: EdgeIt<V, E> curEdge;
      /// ToDo.
      private: EdgeIt<V, E> edgeEnd;
    };

    /// \brief A generic directed graph class.
    /// Both vertexes and edges can store user information. A vertex could be
    /// created passing a custom Id if needed, otherwise it will be choosen
    /// internally. The vertexes also have a name that could be reused among
    /// other vertexes if needed.
    template<typename V, typename E>
    class DirectedGraph
    {
      /// \brief Default constructor.
      public: DirectedGraph() = default;

      /// \brief Constructor.
      /// \param[in] _vertexes Collection of vertexes.
      /// \param[in] _edges Collection of edges.
      public: DirectedGraph(const std::vector<Vertex<V>> &_vertexes,
                            const std::vector<EdgeInitializer<E>> &_edges)
      {
        // Add all vertexes.
        for (auto &v : _vertexes)
        {
          if (!this->AddVertex(v.Data(), v.Name(), v.Id()))
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]"
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto &e : _edges)
        {
          if (!this->AddEdge(e.tailId, e.headId, e.data))
          {
            std::cerr << "Invalid edge [" << e.tailId << "," << e.headId << ","
                      << e.data << "]" << std::endl;
          }
        }
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
      public: NodeIt<V, E> begin() const
      {
        return this->data.begin();
      }

      /// \brief Get an iterator pointing to the past-the-end vertex in the
      /// graph.
      public: NodeIt<V, E> end() const
      {
        return this->data.end();
      }

      /// \brief Get all vertexes of the graph with a given name.
      /// \param[in] _name A name.
      /// \return A vector of shared pointers to all vertexes with name == _name
      //public: VertexPtr_V<V> Vertexes(const std::string &_name) const
      //{
      //  VertexPtr_V<V> res;
      //  auto iter = this->names.find(_name);
      //  if (iter != this->names.end())
      //    res = iter->second;
      //  return res;
      //}

      /// \brief Get all edges of the graph.
      /// \return A set of shared pointers to all edges in the graph.
      //public: EdgePtr_S<V, E> Edges() const
      //{
      //  EdgePtr_S<V, E> res;
      //  for (auto const &pair : this->data)
      //    for (auto const &e : pair.second)
      //      res.insert(e);
      //  return res;
      //}

      public: EdgePtr_S<V, E> Edges() const
      {
        GraphEdgeIt<V, E> graphEdgeIt(this->begin(), this->end());

        EdgePtr_S<V, E> res;
        for (; graphEdgeIt.Valid(); ++graphEdgeIt)
          res.insert(*graphEdgeIt.CurEdge());

        return res;
      }

      /// \brief Whether the graph is empty.
      /// \return True when there are no vertexes in the graph or
      /// false otherwise.
      public: bool Empty() const
      {
        return this->data.empty();
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _vertex The vertex to check adjacent vertexes.
      /// \return A vector of vertexes that are adjacents and directly connected
      /// with an edge.
      //public: VertexPtr_V<V> Adjacents(const VertexPtr<V> _vertex)
      //{
      //  VertexPtr_V<V> res;
      //  auto itVertex = std::find_if(this->data.begin(), this->data.end(),
      //         [&_vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
      //         {
      //            return _pair.first == _vertex;
      //         });
      //  if (itVertex != this->data.end())
      //  {
      //    auto edges = itVertex->second;
      //    for (auto edgePtr : edges)
      //      res.push_back(edgePtr->Head());
      //  }
      //
      //  return res;
      //}

      public: AdjIt<V, E> Adjacents(const VertexPtr<V> _vertex)
      {
        // Get a vertex iterator from the vertex Id.
        AdjList<int, double>::iterator vIt = this->Find(_vertex);

        return AdjIt<V, E>(vIt);
      }

      /// \brief Get all neighbors vertexes that are directly connected to
      /// a given vertex.
      /// \param[in] _id The vertex ID to check adjacent vertexes.
      /// \return A vector of vertexes that are adjacents and directly connected
      /// with an edge.
      public: AdjIt<V, E> Adjacents(const int64_t _id)
      {
        return this->Adjacents(this->VertexById(_id));
      }

      /// \brief Get the set of incoming edges to a given vertex.
      /// \param[in] _vertex Pointer to the vertex.
      /// \return The set of incoming edges to a given vertex.
      //public: EdgePtr_S<V, E> Incidents(const VertexPtr<V> _vertex)
      //{
      //  EdgePtr_S<V, E> res;
      //
      //  for (auto pair : this->data)
      //  {
      //    auto edges = pair.second;
      //    for (auto e : edges)
      //    {
      //      if (e->Head() == _vertex)
      //        res.insert(e);
      //    }
      //  }
      //
      //  return res;
      //}

      /// \brief Get the set of incoming edges to a given node.
      /// \param[in] _id The ID of the vertex.
      /// \return The set of incoming edges to a given node.
      //public: EdgePtr_S<V, E> Incidents(const int64_t _id)
      //{
      //  return this->Incidents(this->VertexById(_id));
      //}

      /// \brief Add a new vertex to the graph.
      /// \param[in] _data User data to be stored in the vertex.
      /// \param[in] _name The name of the vertex.
      /// \param[in] _id Optional Id of the vertex. If not set, the Id will be
      /// internally chosen. If the Id is set but has been already used, the
      /// vertex won't be added.
      /// \return Shared pointer to the new vertex created or nullptr if the
      /// Id specified was already used.
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
        this->data[v] = EdgePtr_S<V, E>();

        // Update the map of Ids.
        this->ids[id] = v;
        // Update the map of names.
        this->names[_name].push_back(v);

        return v;
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _tail Pointer to the tail's vertex.
      /// \param[in] _head Pointer to the head's vertex.
      /// \param[in] _data User data stored in the edge.
      /// \return Shared pointer to the new edge created or nullptr if the
      /// edge was not created (e.g. incorrect vertexes).
      public: EdgePtr<V, E> AddEdge(const VertexPtr<V> &_tail,
                                    const VertexPtr<V> &_head,
                                    const E &_data)
      {
        // Find the tail vertex.
        auto itTail = this->data.find(_tail);
        if (itTail == this->data.end())
          return nullptr;

        // Make sure that the head vertex also exists.
        auto itHead = this->data.find(_head);
        if (itHead == this->data.end())
          return nullptr;

        // Check that the edge is not repeated.
        EdgePtr_S<V, E> &edges = itTail->second;
        auto edgeFound = std::find_if(edges.begin(), edges.end(),
                       [&_head](EdgePtr<V, E> _edge)
                       {
                         return _edge->Head() == _head;
                       });
        if (edgeFound != edges.end())
          return nullptr;

        // Create the edge.
        auto edge = std::make_shared<Edge<V, E>>(_tail, _head, _data);

        // Link the new edge.
        itTail->second.insert(edge);

        // Mark the edge as valid.
        edge->valid = true;

        return edge;
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _tailId ID of the tail's vertex.
      /// \param[in] _headId ID of the head's vertex.
      /// \param[in] _data User data stored in the edge.
      /// \return Shared pointer to the new edge.
      public: EdgePtr<V, E> AddEdge(const int64_t _tailId,
                                    const int64_t _headId,
                                    const E &_data)
      {
        return this->AddEdge(this->VertexById(_tailId),
          this->VertexById(_headId), _data);
      }

      /// \brief ToDo.
      typename AdjList<V, E>::iterator Find(const VertexPtr<V> &_vertex)
      {
        return this->data.find(_vertex);
      }

      /// \brief ToDo.
      typename AdjList<V, E>::iterator Find(const int64_t _vertexId)
      {
        return this->Find(this->VertexById(_vertexId));
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertexes from the edge. Any
      /// call to Tail() or Head() will return nullptr.
      /// \param[in] _edge Pointer to the edge to be removed.
      //public: void RemoveEdge(EdgePtr<V, E> &_edge)
      //{
      //  if (!_edge)
      //    return;
      //
      //  auto vertex = _edge->Tail();
      //  auto itPair = std::find_if(this->data.begin(), this->data.end(),
      //         [&vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
      //         {
      //            return _pair.first == vertex;
      //         });
      //  if (itPair == this->data.end())
      //    return;
      //
      //  EdgePtr_S<V, E> &edges = itPair->second;
      //  edges.erase(_edge);
      //
      //  // Mark the edge as invalid. This will prevent to reach any vertexes if
      //  // there are any shared pointers keeping the edge alive.
      //  _edge->valid = false;
      //}

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex Pointer to the vertex to be removed.
      //public: void RemoveVertex(VertexPtr<V> &_vertex)
      //{
      //  if (!_vertex)
      //    return;
      //
      //  // Save the Id.
      //  auto id = _vertex->Id();
      //
      //  // Remove incident edges.
      //  auto incidents = this->Incidents(_vertex);
      //  for (auto e : incidents)
      //    this->RemoveEdge(e);
      //
      //  // Remove the vertex and all outcoming edges.
      //  auto itPair = std::find_if(this->data.begin(), this->data.end(),
      //         [&_vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
      //         {
      //            return _pair.first == _vertex;
      //         });
      //  if (itPair == this->data.end())
      //    return;
      //
      //  this->data.erase(itPair);
      //
      //  // Remove also the id from the map of Ids.
      //  this->ids.erase(id);
      //
      //  // Remove also the vertex from the map of names.
      //  std::string name = _vertex->Name();
      //  assert(this->names.find(name) != this->names.end());
      //
      //  auto &v = this->names.at(name);
      //  v.erase(std::remove(v.begin(), v.end(), _vertex), v.end());
      //  if (v.empty())
      //    this->names.erase(name);
      //}

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _id ID of the vertex to be removed.
      //public: void RemoveVertex(const int64_t _id)
      //{
      //  auto vPtr = this->VertexById(_id);
      //  if (vPtr)
      //    this->RemoveVertex(vPtr);
      //}

      /// \brief Remove all vertexes with name == _name.
      /// \param[in] _name Name of the vertexes to be removed.
      //public: void RemoveVertexes(const std::string &_name)
      //{
      //  auto iter = this->names.find(_name);
      //  if (iter == this->names.end())
      //    return;
      //
      //  auto &v = iter->second;
      //  while (!v.empty())
      //    this->RemoveVertex(v.front());
      //}

      /// \brief Stream insertion operator.
      /// \param[out] _out The output stream.
      /// \param[in] _g Graph to write to the stream.
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const DirectedGraph<V, E> &_g)
      {
        _out << "Vertexes" << std::endl;
        for (auto const &v : _g.Vertexes())
          _out << "  [" << v->Id() << "][" << v->Name() << "]" << std::endl;

        _out << "Edges" << std::endl;
        for (auto const &e : _g.Edges())
        {
          _out << "  [" << e->Tail()->Id() << "-->"
               << e->Head()->Id() << "]" << std::endl;
        }
        return _out;
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
      protected: AdjList<V, E> data;

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
