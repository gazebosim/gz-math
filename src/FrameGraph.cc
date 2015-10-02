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
void FrameGraph::AddFrame(const std::string &_path,
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
  const auto &srcFrameParent = this->dataPtr->FrameFromAbsolutePath(path);
  auto f = srcFrameParent.lock();

  if (!f)
  {
    std::stringstream ss;
    ss << "Error: parent path \"" << _path << "\" does not exists";
    throw FrameException(ss.str());
  }

  auto it = f->dataPtr->children.find(_name);
  if (it != f->dataPtr->children.end())
  {
    std::stringstream ss;
    ss << "Error: path \"" << _name << "\" already exists";
    throw FrameException(ss.str());
  }

  FramePtr frame(new Frame(_name, _pose, f));
  f->dataPtr->children[_name] = frame;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const std::string &_srcFramePath,
                      const std::string &_dstFramePath) const
{
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

  Pose3d r;
  for (auto &f : up)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      r += p;
    }
  }
  for (auto &f : down)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      r -= p;
    }
  }
  return r;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const FrameWeakPtr &_frame) const
{
  Pose3d p;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto f = _frame.lock();
  if (f)
    p = f->dataPtr->pose;

  return p;
}

/////////////////////////////////////////////////
void FrameGraph::SetPose(FrameWeakPtr _frame, const Pose3d &_p)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto f = _frame.lock();
  if (f)
    f->dataPtr->pose = _p;
}

/////////////////////////////////////////////////
RelativePose FrameGraph::RelativePoses(const std::string &_srcPath,
                                       const std::string &_dstPath) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame,
                                                              _dstPath);
  RelativePose r(srcFrame, dstFrame);
  return r;
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::FrameAccess(const std::string &_path) const
{
  PathPrivate path(_path);
  return this->dataPtr->FrameFromAbsolutePath(path);
}

/////////////////////////////////////////////////
Frame::Frame(const std::string &_name,
             const Pose3d &_pose,
             const FrameWeakPtr &_parentFrame)
  : dataPtr(new FramePrivate(_name, _pose, _parentFrame))
{
}

/////////////////////////////////////////////////
Frame::~Frame()
{
  delete this->dataPtr;
}


/////////////////////////////////////////////////
Frame::Frame(const Frame &_other)
  : dataPtr(new FramePrivate(_other.dataPtr->name, _other.dataPtr->pose,
    FrameWeakPtr()))
{
  *this->dataPtr = *_other.dataPtr;
}

/////////////////////////////////////////////////
Frame &Frame::operator=(const Frame &_other)
{
  if (this == &_other)
    return *this;

  *this->dataPtr = *_other.dataPtr;

  return *this;
}

/////////////////////////////////////////////////
FrameWeakPtr Frame::ParentFrame() const
{
  return this->dataPtr->parentFrame;
}

/////////////////////////////////////////////////
std::string Frame::Name() const
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
  :dataPtr(new RelativePosePrivate())
{
  *this->dataPtr = *_other.dataPtr;
}

/////////////////////////////////////////////////
RelativePose& RelativePose::operator=(const RelativePose &_other)
{
  if (this == &_other)
    return *this;

  *this->dataPtr = *_other.dataPtr;

  return *this;
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const FrameWeakPtr &_srcFrame,
                           const FrameWeakPtr &_dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  auto frame = _srcFrame.lock();
  while (frame && frame->ParentFrame().lock())
  {
    this->dataPtr->up.push_back(frame);
    frame = frame->ParentFrame().lock();
  }
  frame = _dstFrame.lock();
  while (frame && frame->ParentFrame().lock())
  {
    this->dataPtr->down.push_back(frame);
    frame = frame->ParentFrame().lock();
  }
}

/////////////////////////////////////////////////
RelativePose::~RelativePose()
{
  delete this->dataPtr;
}
