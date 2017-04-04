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
#include <set>
#include <vector>

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

      /// \brief Constructor.
      /// \param[in] _id Id of the edge.
      /// \param[in] _vertices The set of Ids of the vertices.
      /// \param[in] _data User data to be stored in the edge.
      public: UndirectedEdge(const EdgeId &_id,
                             const VertexId &_tail,
                             const VertexId &_head,
                             const E &_data)
        : Edge<E>(_id, _tail, _head, _data)
      {
      }
    };

    /// \brief An invalid undirected edge.
    template<typename E>
    UndirectedEdge<E> UndirectedEdge<E>::NullEdge(kNullId,
        kNullId, kNullId, E());

    /// \brief A generic graph class using undirected edges.
    template<typename V, typename E>
    class UndirectedGraph : public Graph<V, E, UndirectedEdge<E>>
    {
      /// \brief Default constructor.
      public: UndirectedGraph() = default;

      public: UndirectedGraph(const std::vector<Vertex<V>> &_vertices,
                            const std::vector<EdgeInitializer<E>> &_edges)
              : Graph<V, E, UndirectedEdge<E>>(_vertices, _edges)
      {
      }
    };
  }
}
#endif
