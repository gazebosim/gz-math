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
#ifndef IGNITION_MATH_GRAPHUNDIRECTED_HH_
#define IGNITION_MATH_GRAPHUNDIRECTED_HH_

#include <algorithm>
#include <iostream>

#include "ignition/math/Graph.hh"

namespace ignition
{
  namespace math
  {
    /// \brief An undirected edge represents a connection between two vertices.
    template<typename E>
    class UndirectedEdge : public Edge<E>
    {
      /// \brief An invalid undirected edge.
      public: static UndirectedEdge<E> NullEdge;

      // Documentation inherited.
      using Edge<E>::Edge;

      // Documentation inherited.
      public: VertexId From(const VertexId &_from) const override
      {
        if (!this->Valid())
          return kNullId;

        assert(this->vertices.size() == 2u);
        if (std::find(this->vertices.begin(), this->vertices.end(), _from) ==
              this->vertices.end())
        {
          return kNullId;
        }

        if (this->vertices.front() == _from)
          return this->vertices.back();

        return this->vertices.front();
      }

      // Documentation inherited.
      public: VertexId To(const VertexId &_to) const override
      {
        return this->From(_to);
      }

      /// \brief Stream insertion operator. The output uses DOT graph
      /// description language.
      /// \param[out] _out The output stream.
      /// \param[in] _e Edge to write to the stream.
      /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const UndirectedEdge<E> &_e)
      {
        auto vertices = _e.Vertices();
        auto it = vertices.begin();
        _out << "  " << *it << " -- ";
        ++it;
        _out << *it << " [label=" << _e.Weight(*it) << "];" << std::endl;
        return _out;
      }
    };

    /// \brief An invalid undirected edge.
    template<typename E>
    UndirectedEdge<E> UndirectedEdge<E>::NullEdge(
      kNullId, 1.0, {kNullId, kNullId}, E());

    /// \brief A generic graph class using undirected edges.
    template<typename V, typename E>
    class UndirectedGraph : public Graph<V, E, UndirectedEdge<E>>
    {
      // Documentation inherited.
      using Graph<V, E, UndirectedEdge<E>>::Graph;

      /// \brief Stream insertion operator. The output uses DOT graph
      /// description language.
      /// \param[out] _out The output stream.
      /// \param[in] _g Graph to write to the stream.
      /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const UndirectedGraph<V, E> &_g)
      {
        _out << "graph {" << std::endl;

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
