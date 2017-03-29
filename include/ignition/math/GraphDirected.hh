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
    class DirectedEdge : public Edge<E>
    {
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
          if (this->AddVertex(v.Name(), v.Id(), v.Data()) < 0)
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring"
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if (this->AddEdge(e.tailId, e.headId, e.data) < 0)
          {
            std::cerr << "Invalid edge [" << e.tailId << "," << e.headId << ","
                      << e.data << "]. Ignoring." << std::endl;
          }
        }
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
