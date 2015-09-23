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
}

/////////////////////////////////////////////////
FrameGraph::FrameGraph(const FrameGraph &_copy)
  : dataPtr(new FrameGraphPrivate())
{
  // private method... prevents copying of FrameGraph
}

/////////////////////////////////////////////////
FrameGraph &FrameGraph::operator=(const FrameGraph &_assign)
{
  // private method... prevents copying of FrameGraph
}

/////////////////////////////////////////////////
void FrameGraph::AddFrame( const std::string &_path,
                           const std::string &_name,
                           const Pose3d &_pose)
{
  // _name, _pose, _relative
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Is it a good name?
  if (!PathPrivate::CheckName(_name))
  {
    std::stringstream ss;
    ss << "The frame \"" << _name
      << "\" is not a valid";
    throw FrameException(ss.str());
  }
  // In a good path?
  PathPrivate path(_path);
  if (!path.IsFull())
  {
    std::stringstream ss;
    ss << "Error adding frame: path \"" << _path
      << "\" is not a fully qualified path";
    throw FrameException(ss.str());
  }
  // remove last path element, since it does not exist yet
  auto &srcFrameParent = this->dataPtr->FrameFromAbsolutePath(path);

  auto it = srcFrameParent.dataPtr->children.find(_name);
  if (it != srcFrameParent.dataPtr->children.end())
  {
    std::stringstream ss;
    ss << "Error: path \"" << _name << "\" already exists";
    throw FrameException(ss.str());
  }
  Frame *frame = new Frame(_name, _pose, &srcFrameParent);
  srcFrameParent.dataPtr->children[_name] = frame;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const std::string &_srcFramePath,
                      const std::string &_dstFramePath) const
{
//  std::cout << "FrameGraph::Pose " << _srcFramePath << " " << _dstFramePath << std::endl;
  RelativePose r = this->RelativePoses(_srcFramePath, _dstFramePath);
  return this->Pose(r);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // the list of frames to traverse are kept in 2 vectors
  const auto &up = _relativePose.dataPtr->up;
  const auto &down = _relativePose.dataPtr->down;

std::cout << "\nCOMPUTE up:" << up.size() << " down:" << down.size() << std::endl;

  Pose3d r;
  for (auto frame : up)
  {
    Pose3d p = frame->dataPtr->pose;
std::cout << " + [" << frame->Name() << "]: " << p << std::endl;
    r += p;
  }
  for (auto frame : down)
  {
    Pose3d p = frame->dataPtr->pose;
std::cout << " - [" << frame->Name() << "]: " << p << std::endl;
    r -= p;
  }
std::cout << " result: " << r << std::endl << std::endl;
  return r;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const Frame& _frame) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return _frame.dataPtr->pose;
}

/////////////////////////////////////////////////
void FrameGraph::Pose(const Frame& _frame, const Pose3d &_p)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  _frame.dataPtr->pose = _p;
}

/////////////////////////////////////////////////
RelativePose FrameGraph::RelativePoses(const std::string &_srcPath,
                                       const std::string &_dstPath) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(&srcFrame,
                                                              _dstPath);
  RelativePose r(&srcFrame, &dstFrame);
  return r;
}

/////////////////////////////////////////////////
Frame &FrameGraph::FrameAccess(const std::string &_path) const
{
  PathPrivate path(_path);
  return this->dataPtr->FrameFromAbsolutePath(path);
}

/////////////////////////////////////////////////
Frame::Frame(const std::string &_name,
             const Pose3d &_pose,
             Frame *_parentFrame)
  :dataPtr(new FramePrivate(_name, _pose, _parentFrame))
{

}

/////////////////////////////////////////////////
Frame::~Frame()
{
  delete this->dataPtr;
}


/////////////////////////////////////////////////
Frame::Frame(const Frame &_other)
{
  this->dataPtr->name = _other.dataPtr->name;
  this->dataPtr->pose = _other.dataPtr->pose;
  this->dataPtr->children = _other.dataPtr->children;
}

/////////////////////////////////////////////////
Frame &Frame::operator = (const Frame &_other)
{
  this->dataPtr->name = _other.dataPtr->name;
  this->dataPtr->pose = _other.dataPtr->pose;
  this->dataPtr->children = _other.dataPtr->children;
  return *this;
}

/////////////////////////////////////////////////
const Frame* Frame::ParentFrame() const
{
  return this->dataPtr->parentFrame;
}

/////////////////////////////////////////////////
const std::string &Frame::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const RelativePose &_other)
{
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
}

/////////////////////////////////////////////////
RelativePose& RelativePose::operator = (const RelativePose &_other)
{
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
  return *this;
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const Frame* _srcFrame,
                           const Frame* _dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  auto frame = _srcFrame;
  while (frame && frame->ParentFrame())
  {
    this->dataPtr->up.push_back(frame);
    frame = frame->ParentFrame();
  }
  frame = _dstFrame;
  while (frame && frame->ParentFrame())
  {
    this->dataPtr->down.push_back(frame);
    frame = frame->ParentFrame();
  }
}

/////////////////////////////////////////////////
RelativePose::~RelativePose()
{
  delete this->dataPtr;
}

