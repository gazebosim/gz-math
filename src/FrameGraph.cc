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

#include <sstream>

#include "ignition/math/FrameGraph.hh"
#include "ignition/math/FrameGraphPrivate.hh"

using namespace ignition;
using namespace math;

#define gzerr std::cerr
#define gzinfo std::cout

class Tokens
{
  public: Tokens(const std::string &_s)
  {
    std::stringstream ss(_s);
    std::string item;
    while (std::getline(ss, item, '/'))
    {
      if (item == "")
        continue;
      if (item == ".")
        continue;
      this->pathElems.push_back(item);
    }
  }

  public: const std::vector<std::string> & Elems() const
  {
    return pathElems;
  }

  public: const std::string &Root() const
  {
    return this->pathElems[0];
  }

  public: const std::string & Leaf() const
  {
    return this->pathElems[this->pathElems.size() -1];
  }

  public: bool IsFull() const
  {
    // Dump();
    // is it empty?
    if (this->pathElems.size() == 0)
    {
      return false;
    }
    // does it start with world?
    if (this->pathElems[0] != "world")
    {
      return false;
    }
    for (const std::string &s : this->pathElems)
    {
      if (s == "..")
      {
        return false;
      }
    }
    return true;
  }

  private: void Dump() const
  {
    int i = 0;
    for (const std::string &e : this->pathElems)
    {
      gzerr << i << "] " << e << std::endl;
      i++;
    }
  }

  private: std::vector<std::string> pathElems;

};

/////////////////////////////////////////////////
FrameGraph::FrameGraph()
  :dataPtr(new FrameGraphPrivate())
{
}

/////////////////////////////////////////////////
FrameGraph::~FrameGraph()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
FrameGraph::FrameGraph(const FrameGraph &_copy)
  : dataPtr(new FrameGraphPrivate())
{
  // private method... should not be called
}

/////////////////////////////////////////////////
FrameGraph &FrameGraph::operator=(const FrameGraph &_assign)
{
  // private method... should not be called
}

/////////////////////////////////////////////////
bool FrameGraph::AddFrame( const std::string &_name,
                           const Pose3d &_pose,
                           const std::string &_parent)
{
  std::cout << "AddFrame "
    << _name << ", "
    << _pose << ", "
    << _parent << ", "
    << std::endl;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Is it a good name?
  Tokens path(_name);
  if (!path.IsFull())
  {
    gzerr << "Error adding frame: path \"" << _name
      << "\" is not a valid, fully qualified path"
      << std::endl;
    return false;
  }
  // we know the path is full and thus it starts with the world frame
  FramePrivate *srcFrame = &this->dataPtr->world;
  for (size_t i=1; i < path.Elems().size() -1; ++i)
  {
    auto &children = srcFrame->children;
    std::string e = path.Elems()[i];
    gzerr << "] *** " << i << ": " << e << std::endl;
    if(children.find(e) == children.end())
    {
      gzerr << "Missing frame element: \"" << e
        << "\" in path \"" << _name << "\""  << std::endl;
      return false;
    }
  }
  FramePrivate *parentFrame = NULL;
  auto frame = new FramePrivate(path.Leaf(), _pose, parentFrame);
  srcFrame->children[path.Leaf()] = frame;
  // success
  return true;
}


/*
  // does it already exist?
  if (this->dataPtr->frames.find(_parent) == this->dataPtr->frames.end())
  {
    std::cerr << "Frame \"" << _name << "\" already exists" << std::endl;
    return false;
  }

  Frame *parentFrame = &this->dataPtr->unknownFrame;
  // look for the parent frame.
  auto it = this->dataPtr->frames.find(_parent);
  if(it != this->dataPtr->frames.end())
  {
      parentFrame = it->second;
  }

  auto frame = new Frame(_name, _parent, _pose, parentFrame);
  this->dataPtr->frames[_name] = frame;
*/

/////////////////////////////////////////////////
bool FrameGraph::Pose(const std::string &_srcFrame,
                      const std::string &_dstFrame,
                      Pose3d &_result) const
{

  std::cout << "FrameGraph::Pose " << _srcFrame << " " << _dstFrame << std::endl;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (true)
    return false;
/*
  // find frames
  auto src = this->dataPtr->frames.find(_srcFrame);
  auto dst = this->dataPtr->frames.find(_dstFrame);

  if(src == this->dataPtr->frames.end())
  {
    std::cerr << "Can't find frame \"" << _srcFrame << "\"" << std::endl;
    return false;
  }

  if(dst == this->dataPtr->frames.end())
  {
    std::cerr << "Can't find frame \"" << _dstFrame << "\"" << std::endl;
    return false;
  }

  Frame &sframe = *src->second;
  Frame &dframe = *dst->second;

  std::cout << "sframe << " << sframe.pose << std::endl;
  std::cout << "dframe << " << dframe.pose << std::endl;

  _result =  dframe.pose + sframe.pose;
*/
  return true;
}

/////////////////////////////////////////////////
bool FrameGraph::Parent(const std::string &_frame,
            std::string &_parent, bool canonical) const
{

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::cerr << "FrameGraph::Parent not implemented " << std::endl;
  return false;
}


