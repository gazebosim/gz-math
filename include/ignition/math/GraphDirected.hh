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

#include <iostream>
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
      /// \brief ID of the tail vertex.
      public: VertexId tailId;

      /// \brief ID of the head vertex.
      public: VertexId headId;

      /// \brief User data.
      public: E data;
    };

    /// \brief A directed edge represents a connection between two vertices.
    template<typename E>
    class DirectedEdge : public Edge
    {
      /// \brief An invalid directed edge.
      public: static DirectedEdge<E> NullEdge;

      /// \brief Constructor.
      /// \param[in] _id Id of the edge.
      /// \param[in] _tail Id of the tail vertex.
      /// \param[in] _head Id of the head vertex.
      /// \param[in] _data User data to be stored in the edge.
      public: DirectedEdge(const EdgeId &_id,
                           const VertexId &_tail,
                           const VertexId &_head,
                           const E &_data)
        : Edge(_id),
          tail(_tail),
          head(_head),
          data(_data)
      {
      }

      /// \brief Get the Id of the tail vertex in this edge.
      /// \return An id of the tail vertex in this edge.
      /// \sa Head()
      public: VertexId Tail() const
      {
        return this->tail;
      }

      /// \brief Get the Id of the head vertex in this edge.
      /// \return An id of the head vertex in this edge.
      /// \sa Tail()
      public: VertexId Head() const
      {
        return this->head;
      }

      /// \brief Get a non-mutable reference to the user data stored in the edge
      /// \return The non-mutable reference to the user data stored in the edge.
      public: const E &Data() const
      {
        return this->data;
      }

      /// \brief Get a mutable reference to the user data stored in the edge.
      /// \return The mutable reference to the user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      // Documentation inherited.
      public: VertexId_S Vertices() const
      {
        // if (!this->Valid())
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

      // Documentation inherited.
      public: VertexId To(const VertexId &_to) const
      {
        if (_to != this->Head())
          return kNullId;

        return this->Tail();
      }

      /// \brief Stream insertion operator. The output uses DOT graph
      /// description language.
      /// \param[out] _out The output stream.
      /// \param[in] _e Edge to write to the stream.
      /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const DirectedEdge<E> &_e)
      {
        _out << "  " << _e.Tail() << " -> " << _e.Head() << ";" << std::endl;
        return _out;
      }

      /// \brief The id of the tail vertex.
      private: VertexId tail;

      /// \brief the id of the head vertex.
      private: VertexId head;

      /// \brief User data.
      private: E data;
    };

    /// \brief An invalid directed edge.
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
      public: DirectedGraph(const std::vector<Vertex<V>> &_vertices,
                            const std::vector<DirectEdgeInitializer<E>> &_edges)
      {
        // Add all vertices.
        for (auto const &v : _vertices)
        {
          if (!this->AddVertex(v.Data(), v.Name(), v.Id()).Valid())
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring"
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if (!this->AddEdge(e.tailId, e.headId, e.data).Valid())
          {
            std::cerr << "Invalid edge [" << e.tailId << "," << e.headId << ","
                      << e.data << "]. Ignoring." << std::endl;
          }
        }
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _tail Id of the tail vertex.
      /// \param[in] _head Id of the head vertex.
      /// \param[in] _data User data stored in the edge.
      /// \return Reference to the new edge created or NullEdge if the
      /// edge was not created (e.g. incorrect vertices).
      public: DirectedEdge<E> &AddEdge(const VertexId &_tail,
                                       const VertexId &_head,
                                       const E &_data)
      {
        auto id = this->NextEdgeId();
        DirectedEdge<E> newEdge(id, _tail, _head, _data);
        return this->LinkEdge(std::move(newEdge));
      }

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
        for (auto const &vertexMap : _g.Vertices())
        {
          auto vertex = vertexMap.second.get();
          _out << vertex;
        }

        // All edges.
        for (auto const &edgeMap : _g.Edges())
        {
          auto edge = edgeMap.second.get();
          _out << edge;
        }

        _out << "}" << std::endl;

        return _out;
      }
    };
  }
}
#endif
