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

// int64_t
#include <cstdint>
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
      public: int64_t tailId;

      /// \brief ID of the head's vertex.
      public: int64_t headId;

      /// \brief User data.
      public: E data;
    };

    // Forward declarations.
    template<typename V, typename E>
    class DirectedEdge;

    /// \def DirectedEdgePtr
    /// \brief Shared pointer to an edge.
    template<typename V, typename E>
    using DirectedEdgePtr = std::shared_ptr<DirectedEdge<V, E>>;

    /// \def EdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename V, typename E>
    using DirectedEdgePtr_S = std::set<DirectedEdgePtr<V, E>>;

    /// \brief A directed edge represents a connection between two vertexes.
    template<typename V, typename E>
    class DirectedEdge : public Edge<V>
    {
      /// \brief Constructor.
      /// \param[in] _tail Shared pointer to the tail vertex.
      /// \param[in] _head Shared pointer to the head vertex.
      /// \param[in] _data User data to be stored in the edge.
      public: DirectedEdge(const VertexPtr<V> _tail,
                           const VertexPtr<V> _head,
                           const E &_data)
        : tail(_tail),
          head(_head),
          data(_data)
      {
      }

      /// \brief Helper function for creating an isolated directed edge.
      /// \param[in] _tail Pointer to the vertex in the tail of the edge.
      /// \param[in] _head Pointer to the vertex in the head of the edge.
      /// \param[in] _data User data.
      public: static DirectedEdgePtr<V, E> createEdge(const VertexPtr<V> _tail,
                                                      const VertexPtr<V> _head,
                                                      const E &_data)
      {
        return std::make_shared<DirectedEdge<V, E>>(_tail, _head, _data);
      }

      /// \brief Get a shared pointer to the tail's vertex in this edge.
      /// \return A shared pointer to the tail's vertex in this edge.
      /// \sa Head()
      public: VertexPtr<V> Tail() const
      {
        if (!this->Valid())
          return nullptr;

        return this->tail;
      }

      /// \brief Get a shared pointer to the head's vertex in this edge.
      /// \return A shared pointer to the head's vertex in this edge.
      /// \sa Tail()
      public: VertexPtr<V> Head() const
      {
        if (!this->Valid())
          return nullptr;

        return this->head;
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      // Documentation inherited.
      public: VertexPtr_S<V> Vertexes() const
      {
        if (!this->Valid())
          return {nullptr, nullptr};

        return {this->tail, this->head};
      }

      // Documentation inherited.
      public: VertexPtr<V> To(const VertexPtr<V> &_from) const
      {
        if (_from != this->Tail())
          return nullptr;

        return this->Head();
      }

      /// \brief Shared pointer to the tail's vertex.
      private: VertexPtr<V> tail;

      /// \brief Shared pointer to the head's vertex.
      private: VertexPtr<V> head;

      /// \brief User data.
      private: E data;
    };

    /// \brief A generic graph class using directed edges.
    template<typename V, typename E>
    class DirectedGraph : public Graph<V, E, DirectedEdge<V, E>>
    {
      /// \brief Default constructor.
      public: DirectedGraph() = default;

      /// \brief Constructor.
      /// \param[in] _vertexes Collection of vertexes.
      /// \param[in] _edges Collection of edges.
      public: DirectedGraph(const std::vector<Vertex<V>> &_vertexes,
                            const std::vector<DirectEdgeInitializer<E>> &_edges)
      {
        // Add all vertexes.
        for (auto const &v : _vertexes)
        {
          if (!this->AddVertex(v.Data(), v.Name(), v.Id()))
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring"
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if (!this->AddEdge(e.tailId, e.headId, e.data))
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
      /// edge was not created (e.g. incorrect vertexes).
      public: DirectedEdgePtr<V, E> AddEdge(const VertexPtr<V> &_tail,
                                            const VertexPtr<V> &_head,
                                            const E &_data)
      {
        auto newEdgePtr = DirectedEdge<V, E>::createEdge(_tail, _head, _data);
        if (this->LinkEdge(newEdgePtr))
          return newEdgePtr;
        else
          return nullptr;
      }

      ///// \brief Add a new edge to the graph.
      ///// \param[in] _tailId ID of the tail's vertex.
      ///// \param[in] _headId ID of the head's vertex.
      ///// \param[in] _data User data stored in the edge.
      ///// \return Shared pointer to the new edge.
      public: DirectedEdgePtr<V, E> AddEdge(const int64_t _tailId,
                                            const int64_t _headId,
                                            const E &_data)
      {
        auto tailPtr = this->VertexById(_tailId);
        auto headPtr = this->VertexById(_headId);
        return this->AddEdge(tailPtr, headPtr, _data);
      }

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
          _out << "  [" << e->Tail()->Id() << "]-->"
               << "["   << e->Head()->Id() << "]" << std::endl;
        }
        return _out;
      }
    };
  }
}
#endif
