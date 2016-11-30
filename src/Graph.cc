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
#include <set>

#include "ignition/math/Vertex.hh"
#include "ignition/math/Graph.hh"

using namespace ignition;
using namespace math;

/// \brief Private graph data.
class ignition::math::GraphPrivate
{
  /// \brief The set of vertices in the graph
  public: std::vector<Vertex> vertices;

  /// \brief The set of edges in the graph
  public: std::vector<std::vector<int>> edges;
};

/////////////////////////////////////////////////
Graph::Graph()
: dataPtr(new GraphPrivate)
{
}

/////////////////////////////////////////////////
Graph::Graph(const Vertex_L &_vertices, const Edge_L &_edges)
: dataPtr(new GraphPrivate)
{
  this->Init(_vertices, _edges);
}

/////////////////////////////////////////////////
Graph::~Graph()
{
}

/////////////////////////////////////////////////
void Graph::ClearEdges()
{
  this->dataPtr->edges.clear();

  // Resize edges so that there is one element per vertex.
  this->dataPtr->edges.resize(this->dataPtr->vertices.size());
}

/////////////////////////////////////////////////
void Graph::Init(const Vertex_L &_vertices, const Edge_L &_edges)
{
  // Clear the graph
  this->dataPtr->vertices.clear();
  this->dataPtr->edges.clear();

  // Add all the vertices
  for (auto const &vertex : _vertices)
  {
    this->dataPtr->vertices.push_back(Vertex<T>(vertex.first, vertex.second));
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

/////////////////////////////////////////////////
std::pair<const Vertex<T> &, bool> Graph::Vertex(const std::string &_name) const
{
  for (auto const &vertex : this->dataPtr->vertices)
  {
    if (vertex.Name() == _name)
      return std::make_pair(vertex, true);
  }
  return ();
}

/////////////////////////////////////////////////
int Graph::VertexIndex(const std::string &_name) const
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

/////////////////////////////////////////////////
const Vertex &Graph::AddVertex(const std::string &_name, const Pose3d &_pose)
{
  this->dataPtr->vertices.push_back(Vertex(_name, _pose));

  // Resize edges so that there is one element per vertex.
  this->dataPtr->edges.resize(this->dataPtr->vertices.size());

  return this->dataPtr->vertices.back();
}

/////////////////////////////////////////////////
const Vertex &Graph::AddVertex(const Vertex &_vertex)
{
  this->dataPtr->vertices.push_back(Vertex(_vertex));

  // Resize edges so that there is one element per vertex.
  this->dataPtr->edges.resize(this->dataPtr->vertices.size());

  return this->dataPtr->vertices.back();
}

/////////////////////////////////////////////////
bool Graph::AddEdge(const Vertex &_parent, const Vertex &_child)
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

/////////////////////////////////////////////////
bool Graph::AddEdge(const std::string &_parent, const std::string &_child)
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

/////////////////////////////////////////////////
Pose3d Graph::Transform(const std::string &_start, const std::string &_end)
{
  Pose3d result;

  std::list<Vertex> path;
  this->Path(_start, _end, path);
  for (auto const &vertex : path)
  {
    result += vertex.Pose();
  }

  return result;
}

/////////////////////////////////////////////////
bool Graph::Path(const std::string &_start, const std::string &_end,
                      std::list<Vertex> &_result)
{
  _result.clear();

  std::vector<int> minDist(this->dataPtr->vertices.size(), IGN_INT32_MAX);
  std::vector<int> prev(this->dataPtr->vertices.size(), -1);

  int startIndex = this->VertexIndex(_start);
  int endIndex = this->VertexIndex(_end);

  if (startIndex < 0)
  {
    std::cerr << "Start vertex[" << _start << "] Not found\n" << std::endl;
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
      // If the distance from the current vertex to the next is less than the
      // store path distance...
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

/////////////////////////////////////////////////
void Graph::Print(std::ostream &_out) const
{
  _out << "Vertices:" << std::endl;
  for (auto const &vertex : this->dataPtr->vertices)
  {
    _out << vertex << std::endl;
  }

  _out << std::endl << "Edges:" << std::endl;
  for (unsigned int parent = 0; parent < this->dataPtr->edges.size(); ++parent)
  {
    for (unsigned int child = 0; child < this->dataPtr->edges[parent].size();
         ++child)
    {
      std::cout << this->dataPtr->vertices[parent].Name() << " -> " <<
                   this->dataPtr->vertices[
                   this->dataPtr->edges[parent][child]].Name() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
size_t Graph::VertexCount() const
{
  return this->dataPtr->vertices.size();
}

/////////////////////////////////////////////////
size_t Graph::EdgeCount() const
{
  return this->dataPtr->edges.size();
}
