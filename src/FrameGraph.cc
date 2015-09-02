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
                           const std::string &_parent)
{
  std::cout << "AddFrame "
    << _name << ", "
    << _pose << ", "
    << _parent << ", "
    << std::endl;

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

  FramePrivate *srcFrame = this->dataPtr->FrameFromAbsolutePath(path);
  if(!srcFrame)
  {
    gzerr << "Path \"" << _name << "\" is not valid" << std::endl;
    return false;
  }

  PathPrivate rpath(_parent);
  FramePrivate *parentFrame = this->dataPtr->FrameFromRelativePath(srcFrame,
                                                                   rpath);

  const std::string &leaf = path.Leaf();
  FramePrivate *frame = new FramePrivate(leaf, _pose, parentFrame);
  srcFrame->children[leaf] = frame;
  // success
  return true;
}

/////////////////////////////////////////////////
bool FrameGraph::Pose(const std::string &_srcFramePath,
                      const std::string &_dstFramePath,
                      Pose3d &_result) const
{

  std::cout << "FrameGraph::Pose " << _srcFramePath << " " << _dstFramePath << std::endl;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  RelativePose relativePose = this->RelativeFrames(_srcFramePath, _dstFramePath);
  return relativePose.Compute(_result);
}

/////////////////////////////////////////////////
RelativePose FrameGraph::RelativeFrames(const std::string &_srcPath,
                                   const std::string &_dstPath) const
{
  FramePrivate *srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  FramePrivate *dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame, _dstPath);
  RelativePose r(&this->dataPtr->mutex, srcFrame, dstFrame);
  return r;
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

RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{

}

RelativePose::RelativePose(const RelativePose &_other)
{
  this->dataPtr->mutex = _other.dataPtr->mutex;
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
}

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
  :dataPtr(new RelativePosePrivate())  // this is the invalid frame
{
  this->dataPtr->mutex = mutex;
  this->dataPtr->up.push_back(_srcFrame);
  this->dataPtr->down.push_back(_dstFrame);
}

RelativePose::~RelativePose()
{
  delete this->dataPtr;
}


/////////////////////////////////////////////////
bool RelativePose::Compute( Pose3d &_p) const
{
  if(!this->dataPtr->mutex)
    return false;

  std::lock_guard<std::mutex> lock(*this->dataPtr->mutex);
  Pose3d r;
  for (auto p : this->dataPtr->up)
  {
    gzerr << "Applying " << p->name << ": " << p->pose << std::endl;
    r += p->pose;
  }
  for (auto p : this->dataPtr->down)
  {
    gzerr << "Applying " << p->name << ": " << p->pose << std::endl;
    r -= p->pose;
  }
  _p = r;
  return true;
}

