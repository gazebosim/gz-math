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
#include <cassert>
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
    /// \brief Used in the UndirectedGraph constructor for uniform
    /// initialization.
    template<typename E>
    struct UndirectEdgeInitializer
    {
      /// \brief IDs of the vertices.
      public: std::set<int64_t> vertices;

      /// \brief User data.
      public: E data;
    };

    // Forward declarations.
    template<typename V, typename E>
    class UndirectedEdge;

    /// \def UndirectedEdgePtr
    /// \brief Shared pointer to an edge.
    template<typename V, typename E>
    using UndirectedEdgePtr = std::shared_ptr<UndirectedEdge<V, E>>;

    /// \def UndirectedEdgePtr_S
    /// \brief Set of shared pointers to edges.
    template<typename V, typename E>
    using UndirectedEdgePtr_S = std::set<UndirectedEdgePtr<V, E>>;

    /// \brief An undirected edge represents a connection between two vertices.
    template<typename V, typename E>
    class UndirectedEdge : public Edge<V>
    {
      /// \brief Constructor.
      /// \param[in] _vertices The set of pointers to two vertices.
      /// \param[in] _data User data to be stored in the edge.
      public: UndirectedEdge(const VertexPtr_S<V> &_vertices,
                             const E &_data)
        : vertices(_vertices),
          data(_data)
      {
      }

      /// \brief Helper function for creating an isolated undirected edge.
      /// \param[in] _vertices The set of pointers to two vertices.
      /// \param[in] _data User data to be stored in the edge.
      public: static
        UndirectedEdgePtr<V, E> createEdge(const VertexPtr_S<V> &_vertices,
                                           const E &_data)
      {
        return std::make_shared<UndirectedEdge<V, E>>(_vertices, _data);
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      // Documentation inherited.
      public: VertexPtr_S<V> Vertices() const
      {
        if (!this->Valid())
          return {nullptr, nullptr};

        return this->vertices;
      }

      // Documentation inherited.
      public: VertexPtr<V> To(const VertexPtr<V> &_from) const
      {
        if (!this->Valid())
          return nullptr;

        assert(this->vertices.size() == 2u);
        auto search = this->vertices.find(_from);
        if (search == this->vertices.end())
          return nullptr;

        VertexPtr_S<V> diff;
        VertexPtr_S<V> s2 = {_from};
        std::set_difference(this->vertices.begin(), this->vertices.end(),
                            s2.begin(), s2.end(),
                            std::inserter(diff, diff.begin()));

        assert(diff.size() > 0u);
        if (diff.size() >= 2u)
          return nullptr;

        return *diff.begin();
      }

      /// \brief The set of pointers to two vertices.
      private: VertexPtr_S<V> vertices;

      /// \brief User data.
      private: E data;
    };

    /// \brief A generic graph class using undirected edges.
    template<typename V, typename E>
    class UndirectedGraph : public Graph<V, E, UndirectedEdge<V, E>>
    {
      /// \brief Default constructor.
      public: UndirectedGraph() = default;

      /// \brief Constructor.
      /// \param[in] _vertices Collection of vertices.
      /// \param[in] _edges Collection of edges.
      public: UndirectedGraph(const std::vector<Vertex<V>> &_vertices,
                          const std::vector<UndirectEdgeInitializer<E>> &_edges)
      {
        // Add all vertices.
        for (auto const &v : _vertices)
        {
          if (!this->AddVertex(v.Data(), v.Name(), v.Id()))
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring."
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if (!this->AddEdge(e.vertices, e.data))
            std::cerr << "Ignoring edge" << std::endl;
        }
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _vertices The set of pointers to two vertices.
      /// \param[in] _data User data.
      /// \return Shared pointer to the new edge created or nullptr if the
      /// edge was not created (e.g. incorrect vertices).
      public: UndirectedEdgePtr<V, E> AddEdge(const VertexPtr_S<V> &_vertices,
                                              const E &_data)
      {
        auto newEdgePtr = UndirectedEdge<V, E>::createEdge(_vertices, _data);

        if (this->LinkEdge(newEdgePtr))
          return newEdgePtr;
        else
          return nullptr;
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _vertices The set of Ids to two vertices.
      /// \param[in] _data User data.
      /// \return Shared pointer to the new edge.
      public:
        UndirectedEdgePtr<V, E> AddEdge(const std::set<int64_t> &_vertices,
                                        const E &_data)
      {
        VertexPtr_S<V> vertices;
        for (auto const &id : _vertices)
          vertices.insert(this->VertexById(id));

        return this->AddEdge(vertices, _data);
      }

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
        for (auto const &v : _g.Vertices())
        {
            _out << "  " << v->Id() << " [label=\"" << v->Name() << " ("
                 << v->Id() << ")\"];\n";
        }

        // All edges.
        VertexPtr_S<V> verticesUsed;
        for (auto const &e : _g.Edges())
        {
          auto vertices = e->Vertices();
          auto it = vertices.begin();
          _out << "  " << (*it)->Id() << " -- ";
          ++it;
          _out << (*it)->Id() << ";\n";
        }

        _out << "}" << std::endl;

        return _out;
      }
    };
  }
}
#endif
