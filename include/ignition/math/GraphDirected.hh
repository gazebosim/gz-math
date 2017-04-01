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
#ifndef IGNITION_MATH_GRAPHDIRECTED_HH_
#define IGNITION_MATH_GRAPHDIRECTED_HH_

#include <memory>
#include <iostream>
#include <set>
#include <vector>

#include "ignition/math/Graph.hh"

namespace ignition
{
  namespace math
  {
    /// \brief Used in the DirectedGraph constructor for uniform initialization.
    template<typename E>
    struct DirectEdgeInitializer
    {
      /// \brief ID of the tail's vertex.
      public: VertexId tailId;

      /// \brief ID of the head's vertex.
      public: VertexId headId;

      /// \brief User data.
      public: E data;
    };

    // Forward declarations.
    template<typename E>
    class DirectedEdge;

    /// \def DirectedEdgePtr
    /// \brief Shared pointer to an edge.
    template<typename E>
    using DirectedEdgePtr = std::shared_ptr<DirectedEdge<E>>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename E>
    using DirectedEdgePtr_S = std::set<DirectedEdgePtr<E>>;

    /// \brief A directed edge represents a connection between two vertices.
    template<typename E>
    class DirectedEdge : public Edge
    {
      /// \brief Constructor.
      /// \param[in] _tail Shared pointer to the tail vertex.
      /// \param[in] _head Shared pointer to the head vertex.
      /// \param[in] _data User data to be stored in the edge.
      //public: DirectedEdge(const EdgeId _id,
      //                     const VertexPtr<V> _tail,
      //                     const VertexPtr<V> _head,
      //                     const E &_data)
      //  : Edge<V>(_id),
      //    tail(_tail),
      //    head(_head),
      //    data(_data)
      //{
      //}

      /// \brief ToDo.
      public: static DirectedEdge<E> NullEdge;

      /// \brief Constructor.
      /// \param[in] _tail Shared pointer to the tail vertex.
      /// \param[in] _head Shared pointer to the head vertex.
      /// \param[in] _data User data to be stored in the edge.
      public: DirectedEdge(const EdgeId _id,
                           const VertexId _tail,
                           const VertexId _head,
                           const E &_data)
        : Edge(_id),
          tail(_tail),
          head(_head),
          data(_data)
      {
      }

      // Move construct.
      //public: DirectedEdge(DirectedEdge &&_other)
      //  : Edge(_other.Id())
      //   //: tail(std::move(_other.tail)),
      //   //  head(std::move(_other.head)),
      //   //  data(std::move(_other.data))
      //{
      //  std::cout << "move" << std::endl;
      //};

      /// \brief Helper function for creating an isolated directed edge.
      /// \param[in] _tail Pointer to the vertex in the tail of the edge.
      /// \param[in] _head Pointer to the vertex in the head of the edge.
      /// \param[in] _data User data.
      //public: static DirectedEdgePtr<V, E> createEdge(const VertexPtr<V> _tail,
      //                                                const VertexPtr<V> _head,
      //                                                const E &_data)
      //{
      //  auto id = this->_NextEdgeId();
      //  return std::make_shared<DirectedEdge<V, E>>(id, _tail, _head, _data);
      //}

      /// \brief Get a shared pointer to the tail's vertex in this edge.
      /// \return A shared pointer to the tail's vertex in this edge.
      /// \sa Head()
      //public: VertexPtr<V> Tail() const
      //{
      //  if (!this->Valid())
      //    return nullptr;
//
      //  return this->tail;
      //}

      /// \brief Get a shared pointer to the tail's vertex in this edge.
      /// \return A shared pointer to the tail's vertex in this edge.
      /// \sa Head()
      public: VertexId Tail() const
      {
        return this->tail;
      }

      /// \brief Get a shared pointer to the head's vertex in this edge.
      /// \return A shared pointer to the head's vertex in this edge.
      /// \sa Tail()
      public: VertexId Head() const
      {
        return this->head;
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      // Documentation inherited.
      //public: VertexPtr_S<V> Vertices() const
      //{
      //  if (!this->Valid())
      //    return {nullptr, nullptr};
//
      //  return {this->tail, this->head};
      //}

      // Documentation inherited.
      public: VertexId_S _Vertices() const
      {
        //if (!this->Valid())
        //  return {Vertex<V>::NullVertex, Vertex<V>::NullVertex};

        return {this->tail, this->head};
      }

      // Documentation inherited.
      public: VertexId From(const VertexId &_from) const
      {
        if (_from != this->Tail())
          return kNullId;

        return this->Head();
      }

      /// \brief ToDo..
      private: VertexId tail;

      /// \brief ToDo.
      private: VertexId head;

      /// \brief User data.
      private: E data;
    };

    /// \def ToDo.
    /// \brief ToDo.
    template<typename E>
    DirectedEdge<E> DirectedEdge<E>::NullEdge(kNullId, kNullId, kNullId, E());

    /// \brief A generic graph class using directed edges.
    template<typename V, typename E>
    class DirectedGraph : public Graph<V, E, DirectedEdge<E>>
    {
      /// \brief Default constructor.
      public: DirectedGraph() = default;

      /// \brief Constructor.
      /// \param[in] _vertices Collection of vertices.
      /// \param[in] _edges Collection of edges.
      //public: DirectedGraph(const std::vector<Vertex<V>> &_vertices,
      //                      const std::vector<DirectEdgeInitializer<E>> &_edges)
      //{
      //  // Add all vertices.
      //  for (auto const &v : _vertices)
      //  {
      //    if (!this->AddVertex(v.Data(), v.Name(), v.Id()))
      //    {
      //      std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring"
      //                << std::endl;
      //    }
      //  }
//
      //  // Add all edges.
      //  for (auto const &e : _edges)
      //  {
      //    if (!this->AddEdge(e.tailId, e.headId, e.data))
      //    {
      //      std::cerr << "Invalid edge [" << e.tailId << "," << e.headId << ","
      //                << e.data << "]. Ignoring." << std::endl;
      //    }
      //  }
      //}

      /// \brief Constructor.
      /// \param[in] _vertices Collection of vertices.
      /// \param[in] _edges Collection of edges.
      //public: DirectedGraph(const std::vector<Vertex<V>> &_vertices,
      //                      const std::vector<DirectEdgeInitializer<E>> &_edges)
      //{
//
//}

      /// \brief Constructor.
      /// \param[in] _vertices Collection of vertices.
      /// \param[in] _edges Collection of edges.
      public: DirectedGraph(const std::vector<Vertex<V>> &_vertices,
                            const std::vector<DirectEdgeInitializer<E>> &_edges)
      {
        // Add all vertices.
        for (auto const &v : _vertices)
        {
          if ((this->_AddVertex(v.Data(), v.Name(), v.Id())).Id() == kNullId)
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring"
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if ((this->AddEdge(e.tailId, e.headId, e.data)).Id() == kNullId)
          {
            std::cerr << "Invalid edge [" << e.tailId << "," << e.headId << ","
                      << e.data << "]. Ignoring." << std::endl;
          }
        }
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _tail Pointer to the tail's vertex.
      /// \param[in] _head Pointer to the head's vertex.
      /// \param[in] _data User data stored in the edge.
      /// \return Shared pointer to the new edge created or nullptr if the
      /// edge was not created (e.g. incorrect vertices).
      public: DirectedEdge<E> &AddEdge(const VertexId &_tail,
                                       const VertexId &_head,
                                       const E &_data)
      {
        auto id = this->_NextEdgeId();
        //auto newEdgePtr =
        //  std::make_shared<DirectedEdge<E>>(id, _tail, _head, _data);
        DirectedEdge<E> newEdge(id, _tail, _head, _data);
        //auto newEdgePtr = DirectedEdge<V, E>::createEdge(_tail, _head, _data);
        return this->_LinkEdge(std::move(newEdge));
      }

      ///// \brief Add a new edge to the graph.
      ///// \param[in] _tailId ID of the tail's vertex.
      ///// \param[in] _headId ID of the head's vertex.
      ///// \param[in] _data User data stored in the edge.
      ///// \return Shared pointer to the new edge.
      //public: DirectedEdgePtr<V, E> AddEdge(const VertexId _tailId,
      //                                      const VertexId _headId,
      //                                      const E &_data)
      //{
      //  auto tailPtr = this->VertexById(_tailId);
      //  auto headPtr = this->VertexById(_headId);
      //  return this->AddEdge(tailPtr, headPtr, _data);
      //}

      /// \brief Stream insertion operator. The output uses DOT graph
      /// description language.
      /// \param[out] _out The output stream.
      /// \param[in] _g Graph to write to the stream.
      /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const DirectedGraph<V, E> &_g)
      {
        _out << "digraph {" << std::endl;

        // All vertices with the name and Id as a "label" attribute.
        for (auto const &vId : _g._Vertices())
        {
          auto v = _g._VertexById(vId);
          _out << "  " << v.Id() << " [label=\"" << v.Name() << " ("
               << v.Id() << ")\"];\n";
        }

        // All edges.
        VertexPtr_S<V> verticesUsed;
        for (auto const &edgeId : _g._Edges())
        {
          auto edge = _g._EdgeById(edgeId);
          auto tail = _g._VertexById(edge.Tail());
          auto head = _g._VertexById(edge.Head());
          _out << "  " << tail.Id() << " -> " << head.Id() << ";\n";
        }

        _out << "}" << std::endl;

        return _out;
      }
    };
  }
}
#endif
