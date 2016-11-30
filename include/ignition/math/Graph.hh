/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_GRAPH_HH_
#define IGNITION_MATH_GRAPH_HH_

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <ignition/math/Vertex.hh>
#include <ignition/math/Edge.hh>

namespace ignition
{
  namespace math
  {
    /// \def Edge_L
    /// \brief List of edge name pairs (parent, child).
    /// \sa Init()
    using Edge_L = std::list<std::pair<std::string, std::string>>;

    /// \class Graph Graph.hh ignition/math/Graph.hh
    /// \brief The Graph class repsents of set of connected vertices.
    /// Function within Graph allow construction of transforms between
    /// vertices.
    template<typename T>
    class IGNITION_VISIBLE Graph
    {
      /// \brief Constructor
      public: Graph() : dataPtr(new GraphPrivate) {}

      /// \brief Create a graph using a list of vertices and edges.
      /// \param[in] _vertex List of vertices and their values.
      /// \param[in] _edges List of edges.
      public: Graph(const std::list<std::pair<std::string, T>> &_vertices,
                    const Edge_L &_edges) : dataPtr(new GraphPrivate)
      {
        this->Init(_vertices, _edges);
      }

      /// \brief Destructor
      public: virtual ~Graph() {}

      /// \brief Get the number of vertices.
      /// \return Number of vertices in the graph.
      public: size_t VertexCount() const
      {
        return this->dataPtr->vertices.size();
      }

      /// \brief Get the number of edges.
      /// \return Number of edges in the graph.
      public: size_t EdgeCount() const
      {
        return this->dataPtr->edges.size();
      }

      /// \brief Get a vertex by name
      /// \param[in] _name Name of the vertex.
      /// \return Reference to the new vertex. On error, the reference will be
      /// to Vertex::Nan, which has a name of "nan"
      public: const Vertex<T> &Vertex(const std::string &_name) const
      {
        for (auto const &vertex : this->dataPtr->vertices)
        {
          if (vertex.Name() == _name)
            return vertex;
        }
        return Vertex::Nan;
      }

      /// \brief Get the index of a vertex.
      /// \param[in] _name Name of the vertex to find.
      /// \return Index of the vertex. -1 is returned when the vertex is not
      /// found.
      public: int VertexIndex(const std::string &_name) const
      {
        int result = -1;

        // Find the vertex
        for (int i = 0; i < this->dataPtr->vertices.size(); ++i)
        {
          if (this->dataPtr->vertices[i].Name() == _name)
          {
            result = i;
            break;
          }
        }

        return result;
      }

      /// \brief Clear the edges.
      public: void ClearEdges();
      {
        this->dataPtr->edges.clear();

        // Resize edges so that there is one element per vertex.
        this->dataPtr->edges.resize(this->dataPtr->vertices.size());
      }

      /// \brief Initialize a graph using a list of vertices and edges.
      /// \param[in] _vertex List of vertices and their values.
      /// \param[in] _edges List of edges.
      public: void Init(const std::list<std::pair<std::string, T>> &_vertices,
                        const Edge_L &_edges)
      {
        // Clear the graph
        this->dataPtr->vertices.clear();
        this->dataPtr->edges.clear();

        // Add all the vertices
        for (auto const &vertex : _vertices)
        {
          this->dataPtr->vertices.push_back(
              Vertex(vertex.first, vertex.second));
        }

        // Resize edges so that there is one element per vertex.
        this->dataPtr->edges.resize(this->dataPtr->vertices.size());

        for (auto const &edge : _edges)
        {
          int parent = this->VertexIndex(edge.first);
          int child = this->VertexIndex(edge.second);
          if (parent >= 0 && child >= 0)
            this->dataPtr->edges[parent].push_back(child);
        }
      }

      /// \brief Add a new vertex.
      /// \param[in] _name Name of the vertex.
      /// \return Reference to the new vertex.
      public: const Vertex<T> &AddVertex(const std::string &_name)
      {
        this->dataPtr->vertices.push_back(Vertex(_name, _pose));

        // Resize edges so that there is one element per vertex.
        this->dataPtr->edges.resize(this->dataPtr->vertices.size());

        return this->dataPtr->vertices.back();
      }

      /// \brief Add a new vertex.
      /// \param[in] _vertex Reference to the vertex to add.
      /// \return Reference to the new vertex.
      public: const Vertex<T> &AddVertex(const Vertex<T> &_vertex)
      {
        this->dataPtr->vertices.push_back(Vertex(_vertex));

        // Resize edges so that there is one element per vertex.
        this->dataPtr->edges.resize(this->dataPtr->vertices.size());

        return this->dataPtr->vertices.back();
      }

      /// \brief Add a new edge.
      /// \param[in] _parent Reference to the parent vertex
      /// \param[in] _child Reference to the child vertex
      /// \return True if the edge was successfully added
      public: bool AddEdge(const Vertex<T> &_parent, const Vertex<T> &_child)
      {
        int parent = this->VertexIndex(_parent.Name());
        int child = this->VertexIndex(_child.Name());

        if (parent >= 0 && child >= 0)
        {
          this->dataPtr->edges[parent].push_back(child);
          return true;
        }

        return false;
      }

      /// \brief Add a new edge.
      /// \param[in] _parent Name of the parent vertex
      /// \param[in] _child Name of the child vertex
      /// \return True if the edge was successfully added
      public: bool AddEdge(const std::string &_parent,
                           const std::string &_child)
      {
        int parent = this->VertexIndex(_parent);
        int child = this->VertexIndex(_child);

        if (parent >= 0 && child >= 0)
        {
          this->dataPtr->edges[parent].push_back(child);
          return true;
        }

        return false;
      }

      /// \brief Get the shortest path from start to end using Dijstra's
      /// algorithm.
      /// \param[in] _start Name of the start vertex
      /// \param[in] _end Name of the end vertex
      /// \return False if a path was not found.
      public: bool Path(const std::string &_start, const std::string &_end,
                        std::list<Vertex<T>> &_result)
      {
        _result.clear();

        std::vector<int> minDist(this->dataPtr->vertices.size(), IGN_INT32_MAX);
        std::vector<int> prev(this->dataPtr->vertices.size(), -1);

        int startIndex = this->VertiexIndex(_start);
        int endIndex = this->VertexIndex(_end);

        if (startIndex < 0)
        {
          std::cerr << "Start vertex[" << _start
            << "] Not found\n" << std::endl;
          return false;
        }

        if (endIndex < 0)
        {
          std::cerr << "End vertex[" << _end << "] Not found\n" << std::endl;
          return false;
        }

        // Set distance from _start to _start to zero
        minDist[startIndex] = 0;

        std::set<std::pair<int, int>> activeVertices;
        activeVertices.insert({0, startIndex});

        // Loop until the shortest path is found
        while (!activeVertices.empty())
        {
          // Get the next vertex to process, and remove it from the list of
          // vertices to process.
          int where = activeVertices.begin()->second;
          activeVertices.erase(activeVertices.begin());

          // Check if the end vertex was reached.
          if (this->dataPtr->vertices[where].Name() == _end)
          {
            // Populate the result list with the start
            _result.push_back(this->dataPtr->vertices[startIndex]);

            // Add the rest of the path to the result list
            for (auto p : prev)
            {
              // The 'p' list will contain -1 for vertices that were not visited.
              // Break once a -1 is reached.
              if (p >= 0)
                _result.push_back(this->dataPtr->vertices[p]);
              else
                break;
            }
            // We've found the best path, so break out.
            break;
          }

          // Process each edge from the current vertex
          for (auto edge : this->dataPtr->edges[where])
          {
            // If the distance from the current vertex to the next is less than
            // the store path distance...
            if (minDist[where] + 1 < minDist[edge])
            {
              // Add this vertex to the final path.
              prev[where] = edge;

              // Calculate and store the distance
              minDist[edge] = minDist[where] + 1;

              // Using an ordered set data structure. Make sure we remove any
              // existing data about this edge.
              activeVertices.erase({minDist[edge], edge});
              activeVertices.insert({minDist[edge], edge});
            }
          }
        }

        return true;
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _pt Graph to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Graph &_g)
      {
        _g.Print(_out);
        return _out;
      }

      /// \brief Helper function to print a graph
      /// \param[in] _out Output stream
      private: void Print(std::ostream &_out) const
      {
        _out << "Vertices:" << std::endl;
        for (auto const &vertex : this->dataPtr->vertices)
        {
          _out << vertex << std::endl;
        }

        _out << std::endl << "Edges:" << std::endl;
        for (unsigned int parent = 0;
             parent < this->dataPtr->edges.size(); ++parent)
        {
          for (unsigned int child = 0;
               child < this->dataPtr->edges[parent].size(); ++child)
          {
            std::cout << this->dataPtr->vertices[parent].Name() << " -> "
              << this->dataPtr->vertices[
              this->dataPtr->edges[parent][child]].Name() << std::endl;
          }
        }
      }

      /// \brief Private vertex graph data.
      private: class GraphPrivate
      {
        /// \brief The set of vertices in the graph
        public: std::vector<Vertex> vertices;

        /// \brief The set of edges in the graph
        public: std::vector<std::vector<int>> edges;
      };


      /// \internal
      /// \brief Private data structure.
      private: std::unique_ptr<GraphPrivate> dataPtr;
    };
  }
}
#endif
