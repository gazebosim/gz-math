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
#ifndef IGNITION_MATH_GRAPH_EDGE_HH_
#define IGNITION_MATH_GRAPH_EDGE_HH_

// uint64_t
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <set>

#include "ignition/math/graph/Vertex.hh"

namespace ignition
{
namespace math
{
namespace graph
{
  /// \def EdgeId.
  /// \brief The unique Id for an edge.
  using EdgeId = uint64_t;

  /// \brief Used in the Graph constructors for uniform initialization.
  template<typename E>
  struct EdgeInitializer
  {
    /// \brief Constructor.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _data The data stored in the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    EdgeInitializer(const VertexId_P &_vertices,
                    const E &_data = E(),
                    const double _weight = 1)
      : vertices(_vertices),
        data(_data),
        weight(_weight)
    {
    };

    /// \brief IDs of the vertices.
    public: VertexId_P vertices;

    /// \brief User data.
    public: E data;

    /// \brief The weight (cost) of the edge.
    public: double weight = 1;
  };

  struct Weighted {};
  struct Unweighted {};
  struct EdgeWithData {};
  struct EdgeNoData {};

  class EdgeCommon
  {
    public: EdgeCommon() = default;

    /// \brief Constructor.
    /// \param[in] _id Unique id.
    /// \param[in] _vertices The vertices of the edge.
    public: explicit EdgeCommon(const EdgeId &_id,
                                const VertexId_P &_vertices)
      : id(_id),
        vertices(_vertices)
    {
    }

    /// \brief Get the edge Id.
    /// \return The edge Id.
    public: EdgeId Id() const
    {
      return this->id;
    }

    /// \brief Get the two vertices contained in the edge.
    /// \return The two vertices contained in the edge.
    public: VertexId_P Vertices() const
    {
      if (!this->Valid())
        return {kNullId, kNullId};

      return this->vertices;
    }

    /// \brief Get if the edge is valid. An edge is valid if its linked in a
    /// graph and its vertices are reachable.
    /// \return True when the edge is valid or false otherwise (invalid Id).
    public: bool Valid() const
    {
      return this->id != kNullId;
    }

    /// \brief Unique edge Id.
    protected: EdgeId id = kNullId;

    /// \brief The set of Ids of the two vertices.
    protected: VertexId_P vertices;
  };

  /// \brief Generic edge class. An edge has two ends and some constraint
  /// between them. For example, a directed edge only allows traversing the
  /// edge in one direction.
  template<class E, class WeightP = Weighted, class DataP = EdgeWithData>
  class Edge : public EdgeCommon
  {
    /// \brief Constructor.
    /// \param[in] _id Unique id.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    /// \param[in] _data The data stored in the edge.
    public: explicit Edge(const EdgeId &_id,
                          const VertexId_P &_vertices,
                          const double _weight,
                          const E &_data)
      : EdgeCommon(_id, _vertices),
        weight(_weight),
        data(_data)
    {
    }

    /// \brief The cost of traversing the _from end to the other end of the edge
    /// \return The cost.
    public: double Weight() const
    {
      return this->weight;
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

    /// \brief The weight (cost) of the edge.
    private: double weight = 1.0;

    /// \brief User data.
    private: E data;
  };

  /// \brief Generic edge class. An edge has two ends and some constraint
  /// between them. For example, a directed edge only allows traversing the
  /// edge in one direction.
  template<>
  class Edge<void, Weighted, EdgeNoData> : public EdgeCommon
  {
    /// \brief Constructor.
    /// \param[in] _id Unique id.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    public: explicit Edge(const EdgeId &_id,
                          const VertexId_P &_vertices,
                          const double _weight)
      : EdgeCommon(_id, vertices),
        weight(_weight)
    {
    }

    /// \brief The cost of traversing the _from end to the other end of the edge
    /// \return The cost.
    public: double Weight() const
    {
      return 5.0;
    }

    /// \brief The weight (cost) of the edge.
    private: double weight = 1.0;
  };

  /// \brief Generic edge class. An edge has two ends and some constraint
  /// between them. For example, a directed edge only allows traversing the
  /// edge in one direction.
  template<typename E>
  class Edge<E, Unweighted, EdgeWithData> : public EdgeCommon
  {
    /// \brief Constructor.
    /// \param[in] _id Unique id.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    public: explicit Edge(const EdgeId &_id,
                          const VertexId_P &_vertices,
                          const E &_data)
      : EdgeCommon(_id, vertices),
        data(_data)
    {
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

    /// \brief User data.
    private: E data;
  };

  /// \brief Generic edge class. An edge has two ends and some constraint
  /// between them. For example, a directed edge only allows traversing the
  /// edge in one direction.
  template<>
  class Edge<void, Unweighted, EdgeNoData> : public EdgeCommon
  {
    using EdgeCommon::EdgeCommon;
  };

  /// \def EdgeId_S
  /// \brief A set of edge Ids.
  using EdgeId_S = std::set<EdgeId>;

  /// \def EdgeRef_M
  /// \brief A map of edges. The key is the edge Id. The value is a reference
  /// to the edge.
  template<typename EdgeType>
  using EdgeRef_M = std::map<EdgeId, std::reference_wrapper<const EdgeType>>;

  /// \brief An undirected edge represents a connection between two vertices.
  template<class E, class WeightP = Weighted, class DataP = EdgeWithData>
  class UndirectedEdge : public Edge<E, WeightP, DataP>
  {
    /// \brief An invalid undirected edge.
    public: static UndirectedEdge<E, WeightP, DataP> NullEdge;

    using Edge<E, WeightP, DataP>::Edge;

    // Documentation inherited.
    public: VertexId From(const VertexId &_from) const// override
    {
      if (!this->Valid())
        return kNullId;

      if (this->Vertices().first != _from && this->Vertices().second != _from)
        return kNullId;

      if (this->Vertices().first == _from)
        return this->Vertices().second;

      return this->Vertices().first;
    }

    // Documentation inherited.
    public: VertexId To(const VertexId &_to) const// override
    {
      return this->From(_to);
    }

    /// \brief Stream insertion operator. The output uses DOT graph
    /// description language.
    /// \param[out] _out The output stream.
    /// \param[in] _e Edge to write to the stream.
    /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
    public: friend std::ostream &operator<<(std::ostream &_out,
                                  const UndirectedEdge<E, WeightP, DataP> &_e)
    {
      auto vertices = _e.Vertices();
      _out << "  " << vertices.first << " -- " << vertices.second
           << " [label=" << _e.Weight() << "];" << std::endl;
      return _out;
    }
  };

  /// \brief An invalid undirected edge.
  template<typename E, typename WeightP, typename DataP>
  UndirectedEdge<E, WeightP, DataP> UndirectedEdge<E, WeightP, DataP>::NullEdge(
    kNullId, {kNullId, kNullId}, 1.0, E());

  /// \brief A directed edge represents a connection between two vertices.
  template<class E, class WeightP = Weighted, class DataP = EdgeWithData>
  class DirectedEdge : public Edge<E, WeightP, DataP>
  {
    /// \brief An invalid directed edge.
    public: static DirectedEdge<E, WeightP, DataP> NullEdge;

    using Edge<E, WeightP, DataP>::Edge;

    /// \brief Get the Id of the tail vertex in this edge.
    /// \return An id of the tail vertex in this edge.
    /// \sa Head()
    public: VertexId Tail() const
    {
      return this->Vertices().first;
    }

    /// \brief Get the Id of the head vertex in this edge.
    /// \return An id of the head vertex in this edge.
    /// \sa Tail()
    public: VertexId Head() const
    {
      return this->Vertices().second;
    }

    // Documentation inherited.
    public: VertexId From(const VertexId &_from) const// override
    {
      if (_from != this->Tail())
        return kNullId;

      return this->Head();
    }

    // Documentation inherited.
    public: VertexId To(const VertexId &_to) const// override
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
                                    const DirectedEdge<E, WeightP, DataP> &_e)
    {
      _out << "  " << _e.Tail() << " -> " << _e.Head()
           << " [label=" << _e.Weight() << "];" << std::endl;
      return _out;
    }
  };

  /// \brief An invalid directed edge.
  template<typename E, typename WeightP, typename DataP>
  DirectedEdge<E, WeightP, DataP> DirectedEdge<E, WeightP, DataP>::NullEdge(
    kNullId, {kNullId, kNullId}, 1.0, E());
}
}
}
#endif
