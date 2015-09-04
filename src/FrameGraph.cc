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
                           const std::string &_relative)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Is it a good name?
  PathPrivate path(_name);
  if (!path.IsFull())
  {
    gzerr << "Error adding frame: path \"" << _name
      << "\" is not a valid, fully qualified path"
      << std::endl;
    return false;
  }
  // remove last path element, since it does not exist yet
  std::string leaf = path.Leaf();
  path.PopLeaf();
  FramePrivate *srcFrameParent = this->dataPtr->FrameFromAbsolutePath(path);
  if(!srcFrameParent)
  {
    gzerr << "Error: path \"" << _name << "\" is not valid" << std::endl;
    return false;
  }
  if (srcFrameParent->children.find(leaf) != srcFrameParent->children.end())
  {
    gzerr << "Error: path \"" << _name << "\" already exists" << std::endl;
    return false;
  }
  PathPrivate rpath(_relative);
  const FramePrivate *relativeFrame = this->dataPtr->FrameFromRelativePath(
                                                                srcFrameParent,
                                                                rpath);
  FramePrivate *frame = new FramePrivate(leaf, _pose, relativeFrame);
gzerr << "++++ addframe " << " ..." << srcFrameParent->name << "/" << leaf  << std::endl;
  srcFrameParent->children[leaf] = frame;
  // success
  return true;
}

/////////////////////////////////////////////////
bool FrameGraph::Pose(const std::string &_srcFramePath,
                      const std::string &_dstFramePath,
                      Pose3d &_result) const
{
  std::cout << "FrameGraph::Pose " << _srcFramePath << " " << _dstFramePath << std::endl;
  RelativePose relativePose;
  if (!this->RelativePoses(_srcFramePath, _dstFramePath, relativePose))
  {
    return false;
  }
  return relativePose.Compute(_result);
}

/////////////////////////////////////////////////
bool FrameGraph::RelativePoses(const std::string &_srcPath,
                              const std::string &_dstPath,
                              RelativePose &_relativePose) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  if(!srcFrame)
    return false;
  auto dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame, _dstPath);
  if(!dstFrame)
    return false;
  RelativePose r(&this->dataPtr->mutex, srcFrame, dstFrame);
  _relativePose = r;
  return true;
}

/////////////////////////////////////////////////
Pose3d *FrameGraph::FramePose(const std::string &_path) const
{
  PathPrivate path(_path);
  FramePrivate *frame = this->dataPtr->FrameFromAbsolutePath(path);
  if(!frame)
    return NULL;
  return &frame->pose;
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
                           const FramePrivate* _srcFrame,
                           const FramePrivate* _dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  this->dataPtr->mutex = mutex;
  auto frame = _srcFrame;
  while (frame && frame->parentFrame)
  {
    this->dataPtr->up.push_back(frame);
    frame = frame->parentFrame;
  }
  frame = _dstFrame;
  while (frame && frame->parentFrame)
  {
    this->dataPtr->down.push_back(frame);
    frame = frame->parentFrame;
  }
}

/////////////////////////////////////////////////
RelativePose::~RelativePose()
{
  delete this->dataPtr;
}


/////////////////////////////////////////////////
bool RelativePose::Compute( Pose3d &_p) const
{
  // check if relative pose is valid
  if(!this->dataPtr->mutex)
    return false;

  gzerr << "\nCOMPUTE up:" << this->dataPtr->up.size()
        << " down:" << this->dataPtr->down.size()
        << std::endl;
  std::lock_guard<std::mutex> lock(*this->dataPtr->mutex);
  Pose3d r;
  for (auto p : this->dataPtr->up)
  {
    gzerr << "Applying >> " << p->name << ": " << p->pose << std::endl;
    r += p->pose;
  }
  for (auto p : this->dataPtr->down)
  {
    gzerr << "Un applying << " << p->name << ": " << p->pose << std::endl;
    r -= p->pose;
  }
  gzerr << " computed: " << r << std::endl;
  _p = r;
  return true;
}

/*
/////////////////////////////////////////////////
bool FrameGraph::ParentFrame(const std::string &_frame,
            std::string &_parent, bool canonical) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::cerr << "FrameGraph::Parent not implemented " << std::endl;
  return false;
}
*/


