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

#define gzerr std::cerr
#define gzinfo std::cout

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
  std::cout << "FrameGraph::Pose " << _srcFramePath << " " << _dstFramePath << std::endl;
  RelativePose relativePose;
  this->RelativePoses(_srcFramePath, _dstFramePath, relativePose);
  Pose3d r;
  return relativePose.Compute();
}

/////////////////////////////////////////////////
bool FrameGraph::RelativePoses(const std::string &_srcPath,
                              const std::string &_dstPath,
                              RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(&srcFrame, _dstPath);
  RelativePose r(&this->dataPtr->mutex, &srcFrame, &dstFrame);
  _relativePose = r;
  return true;
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
const Pose3d &Frame::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Frame::Pose( const Pose3d &_p)
{
  this->dataPtr->pose = _p;
}

/////////////////////////////////////////////////
RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const RelativePose &_other)
{
  this->dataPtr->mutex = _other.dataPtr->mutex;
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
}

/////////////////////////////////////////////////
RelativePose& RelativePose::operator = (const RelativePose &_other)
{
  this->dataPtr->mutex = _other.dataPtr->mutex;
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
}

/////////////////////////////////////////////////
RelativePose::RelativePose(std::mutex *mutex,
                           const Frame* _srcFrame,
                           const Frame* _dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  this->dataPtr->mutex = mutex;
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


/////////////////////////////////////////////////
Pose3d RelativePose::Compute() const
{
  gzerr << "\nCOMPUTE up:" << this->dataPtr->up.size()
        << " down:" << this->dataPtr->down.size()
        << std::endl;
  std::lock_guard<std::mutex> lock(*this->dataPtr->mutex);
  Pose3d r;
  for (auto p : this->dataPtr->up)
  {
    gzerr << "Applying >> " << p->Name() << ": " << p->Pose() << std::endl;
    r += p->Pose();
  }
  for (auto p : this->dataPtr->down)
  {
    gzerr << "Un applying << " << p->Name() << ": " << p->Pose() << std::endl;
    r -= p->Pose();
  }
  gzerr << " computed: " << r << std::endl;
  return r;
}

