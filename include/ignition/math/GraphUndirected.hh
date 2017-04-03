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
    /// \brief Used in the UndirectedGraph constructor for uniform
    /// initialization.
    template<typename E>
    struct UndirectEdgeInitializer
    {
      /// \brief IDs of the vertices.
      public: std::set<VertexId> vertices;

      /// \brief User data.
      public: E data;
    };

    /// \brief An undirected edge represents a connection between two vertices.
    template<typename E>
    class UndirectedEdge : public Edge
    {
      /// \brief An invalid undirected edge.
      public: static UndirectedEdge<E> NullEdge;

      /// \brief Constructor.
      /// \param[in] _id Id of the edge.
      /// \param[in] _vertices The set of Ids of the vertices.
      /// \param[in] _data User data to be stored in the edge.
      public: UndirectedEdge(const EdgeId &_id,
                             const VertexId_S &_vertices,
                             const E &_data)
        : Edge(_id),
          vertices(_vertices),
          data(_data)
      {
      }

      /// \brief Get the user data stored in the edge.
      /// \return The user data stored in the edge.
      public: E &Data()
      {
        return this->data;
      }

      // Documentation inherited.
      public: VertexId_S Vertices() const
      {
        if (!this->Valid())
          return {kNullId, kNullId};

        return this->vertices;
      }

      // Documentation inherited.
      public: VertexId From(const VertexId &_from) const
      {
        if (!this->Valid())
          return kNullId;

        assert(this->vertices.size() == 2u);
        auto search = this->vertices.find(_from);
        if (search == this->vertices.end())
          return kNullId;

        VertexId_S diff;
        VertexId_S s2 = {_from};
        std::set_difference(this->vertices.begin(), this->vertices.end(),
                            s2.begin(), s2.end(),
                            std::inserter(diff, diff.begin()));

        assert(diff.size() > 0u);
        if (diff.size() >= 2u)
          return kNullId;

        return *diff.begin();
      }

      // Documentation inherited.
      public: VertexId To(const VertexId &_to) const
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
        _out << *it << ";" << std::endl;
        return _out;
      }

      /// \brief The set of Ids of the two vertices.
      private: VertexId_S vertices;

      /// \brief User data.
      private: E data;
    };

    /// \brief An invalid undirected edge.
    template<typename E>
    UndirectedEdge<E> UndirectedEdge<E>::NullEdge(
      kNullId, {kNullId, kNullId}, E());

    /// \brief A generic graph class using undirected edges.
    template<typename V, typename E>
    class UndirectedGraph : public Graph<V, E, UndirectedEdge<E>>
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
          if (!this->AddVertex(v.Data(), v.Name(), v.Id()).Valid())
          {
            std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring."
                      << std::endl;
          }
        }

        // Add all edges.
        for (auto const &e : _edges)
        {
          if (!this->AddEdge(e.vertices, e.data).Valid())
            std::cerr << "Ignoring edge" << std::endl;
        }
      }

      /// \brief Add a new edge to the graph.
      /// \param[in] _vertices The set of Ids of the two vertices.
      /// \param[in] _data User data.
      /// \return Reference to the new edge created or NullEdge if the
      /// edge was not created (e.g. incorrect vertices).
      public: UndirectedEdge<E> &AddEdge(const VertexId_S &_vertices,
                                         const E &_data)
      {
        auto id = this->NextEdgeId();
        UndirectedEdge<E> newEdge(id, _vertices, _data);
        return this->LinkEdge(std::move(newEdge));
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
