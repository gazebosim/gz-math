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
FrameException::FrameException(const std::string &_msg)
  :std::runtime_error(_msg)
{}

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
std::cout << "// AddFrame: " <<  _path << "/[" << _name << "] " << _pose << std::endl;
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
  if (!path.IsAbsolute())
  {
    std::stringstream ss;
    ss << "Error adding frame: path \"" << _path
      << "\" is not a fully qualified path";
    throw FrameException(ss.str());
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
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
  {
    auto f = framePtr.lock();
    if (!f)
    {
      std::stringstream ss;
      ss << "Error: path \"" << _path << "\" does not exists";
      throw FrameException(ss.str());
    }
    name = f->Name();
    parentPtr = f->dataPtr->parentFrame;
  }

  // get the name of the frame to remove
  // remove the frame from its parent
  auto p = parentPtr.lock();
  if (!p)
  {
    std::stringstream ss;
    ss << "Error: path \"" << _path << "\" has no parent";
    throw FrameException(ss.str());
  }
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
  if (f)
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
Pose3d FrameGraph::Pose(const RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // the list of frames to traverse are kept in 2 vectors
  const auto &up = _relativePose.dataPtr->up;
  const auto &down = _relativePose.dataPtr->down;

std::cout << "\n// FrameGraph::Pose" << std::endl;
  Pose3d ups;
  for (auto &f : up)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      ups += p;
std::cout << "// ups (" << frame->Name()  << ") " << p << " : " << ups << std::endl;
    }
  }
  Pose3d downs;
  for (auto &f : down)
  {
    auto frame = f.lock();
    if (frame)
    {
      Pose3d p = frame->dataPtr->pose;
      downs += p;
std::cout << "// downs (" << frame->Name()  << ")" << p << " : " << downs << std::endl;
    }
  }
  Pose3d result = ups - downs;
  // Pose3d result = downs - ups;
std::cout << "// POSE result (ups - downs) = " << result << std::endl;
  return result;
}

/////////////////////////////////////////////////
RelativePose FrameGraph::CreateRelativePose(const std::string &_srcPath,
                                       const std::string &_dstPath) const
{
std::cout << "// FrameGraph::CreateRelativePose " << std::endl;
std::cout << "//   " << _srcPath << ", " << _dstPath << "\n" << std::endl;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame,
                                                              _dstPath);
  // create the relative pose object while we have the mutex lock
  RelativePose r(srcFrame, dstFrame);
  return r;
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const FrameWeakPtr &_srcFrame,
                           const FrameWeakPtr &_dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  // Shared pointer locking is not thread safe. However
  // the FrameGraph mutex is locked during the
  // FrameGraph::CreateRelativePose call. This method call
  // assumes the lock is acquired.

std::cout << "// RelativePose::RelativePose " << std::endl;
  auto frame = _srcFrame.lock();
  while (frame && frame->dataPtr->parentFrame.lock())
  {
std::cout << "// up >> " << frame->Name() << std::endl;
    this->dataPtr->up.push_back(frame);
    frame = frame->dataPtr->parentFrame.lock();
  }
  frame = _dstFrame.lock();
  while (frame && frame->dataPtr->parentFrame.lock())
  {
std::cout << "// down >> " << frame->Name() << std::endl;
    this->dataPtr->down.push_back(frame);
    frame = frame->dataPtr->parentFrame.lock();
  }
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
RelativePose::~RelativePose()
{
  delete this->dataPtr;
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
std::string Frame::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{
}

