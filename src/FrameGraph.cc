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

#include "ignition/math/RelativePosePrivate.hh"
#include "ignition/math/FrameGraphPrivate.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
FrameGraph::FrameGraph()
  :dataPtr(new FrameGraphPrivate)
{
}


/////////////////////////////////////////////////
FrameGraph::~FrameGraph()
{
}


/////////////////////////////////////////////////
void FrameGraph::AddFrame(const std::string &_path,
                          const std::string &_name,
                          const Pose3d &_pose)
{
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
  // Does the path exist?
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrameParent = this->dataPtr->FrameFromAbsolutePath(path);
  auto f = srcFrameParent.lock();
  // is the frame a duplicate?
  auto it = f->dataPtr->children.find(_name);
  if (it != f->dataPtr->children.end())
  {
    std::stringstream ss;
    ss << "Error: path \"" << _name << "\" already exists";
    throw FrameException(ss.str());
  }
  // just add it
  FramePtr frame(new Frame(_name, _pose, f));
  f->dataPtr->children[_name] = frame;
}

/////////////////////////////////////////////////
void FrameGraph::DeleteFrame(const std::string &_path)
{
  PathPrivate path(_path);
  if (!path.IsAbsolute())
  {
    std::stringstream ss;
    ss << "Error deleting frame: path \"" << _path
      << "\" is not a fully qualified path";
    throw FrameException(ss.str());
  }
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  FrameWeakPtr framePtr = this->dataPtr->FrameFromAbsolutePath(path);
  std::string name;
  FrameWeakPtr parentPtr;
  // this scope helps keep the life of f as short as possible
  {
    auto f = framePtr.lock();
    name = f->Name();
    parentPtr = f->dataPtr->parentFrame;
  }
  // get the name of the frame to remove
  // remove the frame from its parent
  auto p = parentPtr.lock();
  p->dataPtr->children.erase(name);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const std::string &_srcFramePath,
                      const std::string &_dstFramePath) const
{
  RelativePose r = this->CreateRelativePose(_srcFramePath, _dstFramePath);
  return this->Pose(r);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::LocalPose(const std::string &_path) const
{
  auto frame = this->FrameAccess(_path);
  auto p = this->LocalPose(frame);
  return p;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::LocalPose(const FrameWeakPtr &_frame) const
{
  Pose3d p;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto f = _frame.lock();
  if (!f)
  {
    std::stringstream ss;
    ss << "Trying to get pose of a deleted frame";
    throw FrameException(ss.str());
  }
  p = f->dataPtr->pose;
  return p;
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::FrameAccess(const std::string &_path) const
{
  PathPrivate path(_path);
  return this->dataPtr->FrameFromAbsolutePath(path);
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::FrameAccess(FrameWeakPtr _frame,
                                     const std::string &_relativepath) const
{
  PathPrivate path(_relativepath);
  return this->dataPtr->FrameFromRelativePath(_frame, path);
}

/////////////////////////////////////////////////
void FrameGraph::SetLocalPose(FrameWeakPtr _frame, const Pose3d &_p)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto f = _frame.lock();
  if (f)
    f->dataPtr->pose = _p;
}

/////////////////////////////////////////////////
void FrameGraph::SetLocalPose(const std::string &_path, const Pose3d &_p)
{
  auto frame = this->FrameAccess(_path);
  this->SetLocalPose(frame, _p);
}

/////////////////////////////////////////////////
RelativePose FrameGraph::CreateRelativePose(const std::string &_srcPath,
                                       const std::string &_dstPath) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame,
                                                              _dstPath);
  // create the relative pose object while we have the mutex lock
  RelativePose r(srcFrame, dstFrame);
  return r;
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // the list of frames to traverse are kept in 2 vectors
  const auto &up = _relativePose.dataPtr->up;
  const auto &down = _relativePose.dataPtr->down;
  // add all the transform from dst to world
  Pose3d ups;
  for (auto &f : up)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      ups += p;
    }
  }
  // add all the transform from src to world (often empty)
  Pose3d downs;
  for (auto &f : down)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      downs += p;
    }
  }
  // apply the opposite of the downs to the ups
  Pose3d result = ups - downs;
  return result;
}
