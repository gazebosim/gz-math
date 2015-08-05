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

#include "ignition/math/FrameGraph.hh"
#include "ignition/math/FrameGraphPrivate.hh"

using namespace ignition;
using namespace math;

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

  std::cout << "AddFrame " << std::endl;

  // look for the parent frame.
  Frame *parentFrame = NULL;
  auto it = this->dataPtr->frames.find(_parent);
  if(it != this->dataPtr->frames.end())
  {
      parentFrame = it->second;
  }
  // create a parent frame
  else
  {
    parentFrame = new Frame(_parent, Pose3d(), NULL);
    this->dataPtr->frames[_parent] = parentFrame;
  }
  auto frame = new Frame(_name, _pose, parentFrame);
  this->dataPtr->frames[_name] =  frame;
}

/////////////////////////////////////////////////
bool FrameGraph::Pose(const std::string &_srcFrame,
                      const std::string &_dstFrame,
                      Pose3d &_result) const
{

  std::cout << "FrameGraph::Pose " << _srcFrame << " " << _dstFrame << std::endl;
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
  return true;
}


