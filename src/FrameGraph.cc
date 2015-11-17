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
    throw FrameException("The frame '" + _name + "' is not a valid");

  // In a good path?
  PathPrivate path(_path);

  // Does the path exist?
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrameParent = this->dataPtr->FrameFromAbsolutePath(path);
  auto f = srcFrameParent.lock();

  // just add it
  if (!f->AddChild(_name, _pose, f))
  {
    // Currently, Frame::AddChild only returns false if the child
    // name already exists.
    throw FrameException("Error: path '" + _name + "' already exists");
  }
}

/////////////////////////////////////////////////
void FrameGraph::DeleteFrame(const std::string &_path)
{
  PathPrivate path(_path);
  if (!path.IsAbsolute())
  {
    throw FrameException("Error deleting frame: path '" + _path +
        "' is not a fully qualified path");
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  FrameWeakPtr framePtr = this->dataPtr->FrameFromAbsolutePath(path);
  std::string name;
  FrameWeakPtr parentPtr;

  // this scope helps keep the life of f as short as possible
  {
    auto f = framePtr.lock();
    name = f->Name();
    parentPtr = f->ParentFrame();
  }

  // get the name of the frame to remove
  // remove the frame from its parent
  auto p = parentPtr.lock();
  p->DeleteChild(name);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const std::string &_dstFramePath,
                        const std::string &_srcFramePath) const
{
  RelativePose r = this->CreateRelativePose(_dstFramePath, _srcFramePath);
  return this->Pose(r);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::LocalPose(const std::string &_path) const
{
  return this->LocalPose(this->Frame(_path));
}

/////////////////////////////////////////////////
Pose3d FrameGraph::LocalPose(const FrameWeakPtr &_frame) const
{
  Pose3d p;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto f = _frame.lock();
  if (!f)
    throw FrameException("Trying to get pose of a deleted frame");
  p = f->Pose();
  return p;
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::Frame(const std::string &_path) const
{
  PathPrivate path(_path);
  return this->dataPtr->FrameFromAbsolutePath(path);
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraph::Frame(FrameWeakPtr _frame,
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
    f->SetPose(_p);
}

/////////////////////////////////////////////////
void FrameGraph::SetLocalPose(const std::string &_path, const Pose3d &_p)
{
  this->SetLocalPose(this->Frame(_path), _p);
}

/////////////////////////////////////////////////
RelativePose FrameGraph::CreateRelativePose(const std::string &_dstPath,
                                            const std::string &_srcPath) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &dstFrame = this->dataPtr->FrameFromAbsolutePath(_dstPath);
  const auto &srcFrame = this->dataPtr->FrameFromRelativePath(dstFrame,
                                                              _srcPath);
  // create the relative pose object while we have the mutex lock
  return RelativePose(dstFrame, srcFrame);
}

/////////////////////////////////////////////////
Pose3d FrameGraph::Pose(const RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // add all the transform from dst to root
  Pose3d ups;
  for (auto const &f : _relativePose.Up())
  {
    auto frame = f.lock();
    if (frame)
      ups += frame->Pose();
  }

  // add all the transform from src to root (often empty)
  Pose3d downs;
  for (auto &f : _relativePose.Down())
  {
    auto frame = f.lock();
    if (frame)
      downs += frame->Pose();
  }

  // apply the opposite of the downs to the ups
  return ups - downs;
}

/////////////////////////////////////////////////
void FrameGraph::Print(std::ostream &_out) const
{
  _out << *this->dataPtr->root;
}
