/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ignition/math/Frame.hh"
#include "ignition/math/FrameGraphPrivate.hh"
#include "ignition/math/FrameGraph.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
FrameGraph::FrameGraph()
: dataPtr(new FrameGraphPrivate)
{
}

/////////////////////////////////////////////////
FrameGraph::FrameGraph(const Frame_L &_frames, const Edge_L &_edges)
  : dataPtr(new FrameGraphPrivate)
{
  this->Init(_frames, _edges);
}

/////////////////////////////////////////////////
FrameGraph::~FrameGraph()
{
}

/////////////////////////////////////////////////
void FrameGraph::Init(const Frame_L &_frames, const Edge_L &_edges)
{
  this->dataPtr->frames.clear();
  this->dataPtr->edges.clear();
  for (auto const &frame : _frames)
  {
    this->dataPtr->frames.push_back(
        FramePtr(new Frame(frame.first, frame.second)));
  }

  // Resize edges so that there is one element per frame.
  this->dataPtr->edges.resize(this->dataPtr->frames.size());

  for (auto const &edge : _edges)
  {
    int parent = this->NodeIndex(edge.first);
    int child = this->NodeIndex(edge.second);
    if (parent >= 0 && child >= 0)
      this->dataPtr->edges[parent].push_back(child);
  }
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::Node(const std::string &_name) const
{
  for (auto const &frame : this->dataPtr->frames)
  {
    if (frame->Name() == _name)
      return frame;
  }
  return FrameWeakPtr();
}

/////////////////////////////////////////////////
int FrameGraph::NodeIndex(const std::string &_name) const
{
  int result = -1;

  // Find the frame
  for (int i = 0; i < this->dataPtr->frames.size(); ++i)
  {
    if (this->dataPtr->frames[i]->Name() == _name)
    {
      result = i;
      break;
    }
  }

  return result;
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::AddNode(const std::string &_name, const Pose3d &_pose)
{
  this->dataPtr->frames.push_back(
      FramePtr(new Frame(_name, _pose)));
}

/////////////////////////////////////////////////
void FrameGraph::AddNode(FramePtr _frame)
{
  this->dataPtr->frames.push_back(_frame);
}

/////////////////////////////////////////////////
void FrameGraph::AddEdge(FramePtr _parent, FramePtr _child)
{
  int parent = this->NodeIndex(_parent->Name());
  int child = this->NodeIndex(_child->Name());

  if (parent >= 0 && child >= 0)
    this->dataPtr->edges[parent].push_back(child);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Transform(const std::string &_start, const std::string &_end)
{
  Pose3d result;

  std::list<FrameWeakPtr> path;
  this->Path(_start, _end, path);
  for (auto const &frame : path)
  {
    result += frame.lock()->Pose();
  }

  return result;
}

/////////////////////////////////////////////////
void FrameGraph::Path(const std::string &_start, const std::string &_end,
                      std::list<FrameWeakPtr> &_result)
{
  _result.clear();

  std::vector<int> minDist(this->dataPtr->frames.size(), IGN_INT32_MAX);
  std::vector<int> prev(this->dataPtr->frames.size(), -1);

  int startIndex = this->NodeIndex(_start);
  int endIndex = this->NodeIndex(_end);

  if (startIndex < 0)
  {
    std::cerr << "Start frame[" << _start << "] Not found\n" << std::endl;
    return;
  }

  if (endIndex < 0)
  {
    std::cerr << "End frame[" << _end << "] Not found\n" << std::endl;
    return;
  }

  // Set distance from _start to _start to zero
  minDist[startIndex] = 0;

  std::set<std::pair<int, int>> activeFrames;
  activeFrames.insert({0, startIndex});

  // Loop until the shortest path is found
  while (!activeFrames.empty())
  {
    // Get the next frame to process, and remove it from the list of frames
    // to process.
    int where = activeFrames.begin()->second;
    activeFrames.erase(activeFrames.begin());

    // Check if the end frame was reached.
    if (this->dataPtr->frames[where]->Name() == _end)
    {
      // Populate the result list with the start
      _result.push_back(this->dataPtr->frames[startIndex]);

      // Add the rest of the path to the result list
      for (auto p : prev)
      {
        // The 'p' list will contain -1 for vertices that were not visited.
        // Break once a -1 is reached.
        if (p >= 0)
          _result.push_back(this->dataPtr->frames[p]);
        else
          break;
      }
      // We've found the best path, so break out.
      break;
    }

    // Process each edge from the current frame
    for (auto edge : this->dataPtr->edges[where])
    {
      // If the distance from the current frame to the next is less than the
      // store path distance...
      if (minDist[where] + 1 < minDist[edge])
      {
        // Add this frame to the final path.
        prev[where] = edge;

        // Calculate and store the distance
        minDist[edge] = minDist[where] + 1;

        // Using an ordered set data structure. Make sure we remove any
        // existing data about this edge.
        activeFrames.erase({minDist[edge], edge});
        activeFrames.insert({minDist[edge], edge});
      }
    }
  }
}

/////////////////////////////////////////////////
void FrameGraph::Print(std::ostream &_out) const
{
  _out << "Frames:" << std::endl;
  for (auto const &frame : this->dataPtr->frames)
  {
    _out << *frame << std::endl;
  }

  _out << std::endl << "Edges:" << std::endl;
  for (unsigned int parent = 0; parent < this->dataPtr->edges.size(); ++parent)
  {
    for (unsigned int child = 0; child < this->dataPtr->edges[parent].size();
         ++child)
    {
      std::cout << this->dataPtr->frames[parent]->Name() << " -> " <<
                   this->dataPtr->frames[
                   this->dataPtr->edges[parent][child]]->Name() << std::endl;
    }
  }
}
