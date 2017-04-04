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
    /// \brief A directed edge represents a connection between two vertices.
    template<typename E>
    class DirectedEdge : public Edge<E>
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
        : Edge<E>(_id, _tail, _head, _data)
      {
        this->ostreamSymbol = "->";
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
    };

    /// \brief An invalid undirected edge.
    template<typename E>
    DirectedEdge<E> DirectedEdge<E>::NullEdge(kNullId, kNullId, kNullId, E());

    /// \brief A generic graph class using directed edges.
    template<typename V, typename E>
    class DirectedGraph : public Graph<V, E, DirectedEdge<E>>
    {
      /// \brief Default constructor.
      public: DirectedGraph() = default;
      public: DirectedGraph(const std::vector<Vertex<V>> &_vertices,
                            const std::vector<EdgeInitializer<E>> &_edges)
              : Graph<V, E, DirectedEdge<E>>(_vertices, _edges)
      {
        this->ostreamName = "digraph";
      }
    };
  }
}
#endif
