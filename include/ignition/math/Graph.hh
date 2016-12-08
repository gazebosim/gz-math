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
#ifndef IGNITION_MATH_GRAPH_HH_
#define IGNITION_MATH_GRAPH_HH_

#include <algorithm>
#include <map>
#include <memory>
#include <set>
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
      /// \param[in] _id Unique id.
      public: Vertex(const V &_data, const long _id)
        : data(_data),
          id(_id)
      {
      }

      /// \brief Retrieve the user information.
      /// \return A mutable reference to the user information.
      public: V &Data()
      {
        return this->data;
      }

      /// \brief Get the vertex Id.
      /// \return The vertex Id.
      public: long Id() const
      {
        return this->id;
      }

      /// \brief User information.
      private: V data;

      /// \brief Unique vertex Id.
      private: long id;
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
    class Graph;

    /// \brief An edge represents a connection between two vertexes.
    template<typename V, typename E>
    class Edge
    {
      /// \brief Constructor.
      /// \param[in] _src Shared pointer to the source vertex.
      /// \param[in] _dst Shared pointer to the destination vertex.
      /// \param[in] _data User data to be stored in the edge.
      public: Edge(const VertexPtr<V> _src,
                   const VertexPtr<V> _dst,
                   const E &_data)
        : src(_src),
          dst(_dst),
          data(_data)
      {
      }

      /// \brief Get a shared pointer to the source's vertex in this edge.
      /// \return A shared pointer to the source's vertex in this edge.
      public: VertexPtr<V> Source() const
      {
        if (!this->valid)
          return nullptr;

        return this->src;
      }

      /// \brief Get a shared pointer to the destination's vertex in this edge.
      /// \return A shared pointer to the destination's vertex in this edge.
      public: VertexPtr<V> Destination() const
      {
        if (!this->valid)
          return nullptr;

        return this->dst;
      }

      /// \brief Get the user data stored in the edge.
      /// \param The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      /// \brief Shared pointer to the source's vertex.
      private: VertexPtr<V> src;

      /// \brief Shared pointer to the destination's vertex.
      private: VertexPtr<V> dst;

      /// \brief User data.
      private: E data;

      /// \brief True when the edge is connected in a graph or false otherwise.
      /// This member variable exists to prevent the following situation:
      ///   Imagine that you have a shared pointer pointing to a valid edge
      ///   connected in a graph. Now, you use the Graph API and remove the
      ///   edge. This operation will disconnect the edge from the graph but the
      ///   edge won't be deallocated because you will keep a shared pointer.
      ///   Having a shared pointer to the edge will allow you to traverse the
      ///   edge and go to any of the vertexes of the graph and modify it.
      /// Once an edge is removed, this flag is set to false and this will
      /// prevent you from traversing the edge and reach the vertexes.
      private: bool valid = false;

      /// The Graph class needs to modify 'valid' during edge removal.
      friend class Graph<V, E>;
    };

    /// \def EdgePtr
    /// \brief Shared pointer to an edge.
    template<typename V, typename E>
    using EdgePtr = std::shared_ptr<Edge<V, E>>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename V, typename E>
    using EdgePtr_S = std::set<EdgePtr<V, E>>;

    /// \brief A generic graph class.
    /// Both vertexes and edges can store user information. A vertex could be
    /// created passing a custom Id if needed, otherwise it will be choosen
    /// internally.
    template<typename V, typename E>
    class Graph
    {
      /// \brief Default constructor.
      public: Graph()
      {
      }

      /// \brief Get a pointer to a vertex using its Id.
      /// \return A shared pointer to the vertex with Id = _id .
      public: VertexPtr<V> VertexById(const long _id)
      {
        if (this->ids.find(_id) == this->ids.end())
          return nullptr;

        return this->ids.at(_id);
      }

      /// \brief Get all vertexes of the graph.
      /// \return A vector of shared pointers to all vertexes in the graph.
      public: VertexPtr_V<V> Vertexes() const
      {
        VertexPtr_V<V> res;
        for (auto const &pair : this->data)
          res.push_back(pair.first);
        return res;
      }

      /// \brief Get all edges of the graph.
      /// \return A set of shared pointers to all edges in the graph.
      public: EdgePtr_S<V, E> Edges() const
      {
        EdgePtr_S<V, E> res;
        for (auto const &pair : this->data)
          for (auto const &e : pair.second)
            res.insert(e);
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
      public: VertexPtr_V<V> Adjacents(const VertexPtr<V> _vertex)
      {
        VertexPtr_V<V> res;
        auto itVertex = std::find_if(this->data.begin(), this->data.end(),
               [&_vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
               {
                  return _pair.first == _vertex;
               });
        if (itVertex != this->data.end())
        {
          auto edges = (*itVertex).second;
          for (auto edgePtr : edges)
            res.push_back(edgePtr->Destination());
        }

        return res;
      }

      /// \brief Get the set of outgoing edges from a given node.
      /// \return The set of outgoing edges from a given node.
      public: EdgePtr_S<V, E> Incidents(const VertexPtr<V> _vertex)
      {
        EdgePtr_S<V, E> res;

        for (auto pair : this->data)
        {
          auto edges = pair.second;
          for (auto e : edges)
          {
            if (e->Destination() == _vertex)
              res.insert(e);
          }
        }

        return res;
      }

      /// \brief Add a new vertex to the graph.
      /// \param[in] _data User data to be stored in the vertex.
      /// \param[in] _id Optional Id of the vertex. If not set, the Id will be
      /// internally chosen. If the Id is set but has been already used, the
      /// vertex won't be added.
      /// \return Shared pointer to the new vertex created or nullptr if the
      /// Id specified was already used.
      public: VertexPtr<V> AddVertex(const V &_data,
                                     const long _id = -1)
      {
        auto id = _id;
        // The user didn't provide an Id, we generate it.
        if (id == -1)
          id = this->NextId();
        // The user provided an Id but already exists.
        else if (this->ids.find(id) != this->ids.end())
          return nullptr;

        // Create the vertex.
        auto v = std::make_shared<Vertex<V>>(_data, id);
        // Link the vertex with an empty list of edges.
        this->data.push_back(std::make_pair(v, EdgePtr_S<V, E>()));
        // Update the map of Ids.
        this->ids[id] = v;

        return v;
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _src Pointer to the source's vertex.
      /// \param[in] _dst Pointer to the destination''s vertex.
      /// \param[in] _data User data stored in the edge.
      public: EdgePtr<V, E> AddEdge(const VertexPtr<V> &_src,
                                    const VertexPtr<V> &_dst,
                                    const E &_data)
      {
        // Find the origin vertex.
        auto itSrc = std::find_if(this->data.begin(), this->data.end(),
                       [&_src](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
                       {
                         return _pair.first == _src;
                       });

        if (itSrc == this->data.end())
          return nullptr;

        // Make sure that the destination vertex also exists.
        auto itDst = std::find_if(this->data.begin(), this->data.end(),
                       [&_dst](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
                       {
                         return _pair.first == _dst;
                       });
        if (itDst == this->data.end())
          return nullptr;

        // Create the edge.
        auto edge = std::make_shared<Edge<V, E>>(_src, _dst, _data);

        // Link the new edge.
        (*itSrc).second.insert(edge);

        // Mark the edge as valid.
        edge->valid = true;

        return edge;
      }

      /// \brief Remove an existing edge from the graph. After the removal, it
      /// won't be possible to reach any of the vertexes from the edge. Any
      /// call to Source() or Destination() will return nullptr.
      /// \param[in] _edge Pointer to the edge to be removed.
      public: void RemoveEdge(EdgePtr<V, E> &_edge)
      {
        auto vertex = _edge->Source();
        auto itPair = std::find_if(this->data.begin(), this->data.end(),
               [&vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
               {
                  return _pair.first == vertex;
               });
        if (itPair == this->data.end())
          return;

        std::set<EdgePtr<V, E>> &edges = (*itPair).second;
        edges.erase(_edge);

        // Mark the edge as invalid. This will prevent to reach any vertexes if
        // there are any shared pointers keeping the edge alive.
        _edge->valid = false;
      }

      /// \brief Remove an existing vertex from the graph.
      /// \param[in] _vertex Pointer to the vertex to be removed.
      public: void RemoveVertex(VertexPtr<V> &_vertex)
      {
        // Save the Id.
        auto id = _vertex->Id();

        // Remove incident edges.
        auto incidents = this->Incidents(_vertex);
        for (auto e : incidents)
          this->RemoveEdge(e);

        // Remove the vertex and all outcoming edges.
        auto itPair = std::find_if(this->data.begin(), this->data.end(),
               [&_vertex](std::pair<VertexPtr<V>, EdgePtr_S<V, E>> _pair)
               {
                  return _pair.first == _vertex;
               });
        this->data.erase(itPair);

        // Remove also the id from the map of Ids.
        this->ids.erase(id);
      }

      /// \brief Stream insertion operator.
      /// \param[out] _out The output stream.
      /// \param[in] _g Graph to write to the stream.
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const Graph<V, E> &_g)
      {
        _out << "Vertices" << std::endl;
        for (auto const &v : _g.Vertexes())
          _out << "  [" << v->Id() << "]" << std::endl;

        _out << "Edges" << std::endl;
        for (auto const &e : _g.Edges())
        {
          _out << "  [" << e->Source()->Id() << "--"
               << e->Destination()->Id() << "]" << std::endl;
        }
        return _out;
      }

      /// \brief Get an available Id to be assigned to a new vertex.
      /// \return The next available Id.
      private: long NextId()
      {
        while (this->ids.find(this->nextId) != this->ids.end())
          ++this->nextId;

        return this->nextId;
      }

      /// The graph is represented using an adjacency list.
      private: std::vector<std::pair<VertexPtr<V>, EdgePtr_S<V, E>>> data;

      /// \brief List of ids curently used.
      private: std::map<long, VertexPtr<V>> ids;

      /// \brief The next vertex Id to be assigned to a new vertex.
      private: long nextId = 0;
    };
  }
}
#endif
