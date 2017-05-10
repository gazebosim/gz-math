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

#include "ignition/math/Graph.hh"

namespace ignition
{
  namespace math
  {
    /// \brief A directed edge represents a connection between two vertices.
    template<typename E>
    class DirectedEdge : public Edge<E>
    {
      /// \brief An invalid directed edge.
      public: static DirectedEdge<E> NullEdge;

      // Documentation inherited.
      using Edge<E>::Edge;

      /// \brief Get the Id of the tail vertex in this edge.
      /// \return An id of the tail vertex in this edge.
      /// \sa Head()
      public: VertexId Tail() const
      {
        return this->vertices.front();
      }

      /// \brief Get the Id of the head vertex in this edge.
      /// \return An id of the head vertex in this edge.
      /// \sa Tail()
      public: VertexId Head() const
      {
        return this->vertices.back();
      }

      // Documentation inherited.
      public: VertexId From(const VertexId &_from) const override
      {
        if (_from != this->Tail())
          return kNullId;

        return this->Head();
      }

      // Documentation inherited.
      public: VertexId To(const VertexId &_to) const override
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
        _out << "  " << _e.Tail() << " -> " << _e.Head()
             << " [label=" << _e.Weight(_e.Tail()) << "];" << std::endl;
        return _out;
      }
    };

    /// \brief An invalid directed edge.
    template<typename E>
    DirectedEdge<E> DirectedEdge<E>::NullEdge(
      kNullId, 1.0, {kNullId, kNullId}, E());

    /// \brief A generic graph class using directed edges.
    template<typename V, typename E>
    class DirectedGraph : public Graph<V, E, DirectedEdge<E>>
    {
      // Documentation inherited.
      using Graph<V, E, DirectedEdge<E>>::Graph;

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
