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
bool FrameGraph::Pose(const std::string &_srcFrame,
                      const std::string &_dstFrame,
                      Pose3d &_result) const
{

  std::cout << "FrameGraph::Pose " << _srcFrame << " " << _dstFrame << std::endl;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &relativePose = this->FrameTransform(_srcFrame, _dstFrame);
  if( relativePose == this->Invalid())
  {
    return false;
  }
  gzerr << "LOOK, its VALID :-)" << std::endl;
  _result = relativePose.Compute();
  return true;
}

/////////////////////////////////////////////////
RelativePose FrameGraph::FrameTransform(const std::string &_srcFrame,
                                   const std::string &_dstFrame) const
{
  gzerr << "INVALID!!!" << std::endl;
  return this->Invalid();
}

/*
bool operator != (RelativePose const &_a, RelativePose const &_b)
{
  return !(_a == _b);
}

bool operator == (RelativePose const &_a, RelativePose const &_b)
{
  return &_a == &_b;
}
*/

bool ignition::math::operator != (const ignition::math::RelativePose &_a,
                                const ignition::math::RelativePose &_b)
{
  // defer to operator ==
  return !(_a == _b);
}


bool ignition::math::operator == (const ignition::math::RelativePose &_a,
                                const ignition::math::RelativePose &_b)
{
  // compare pointer values
  bool r;
  // invalid RelativePose have a dataPtr NULL
  if(_a.dataPtr ==  NULL && _b.dataPtr == NULL)
    r = true;
  r = false;
  gzerr << "A dataptr  @" << _a.dataPtr << " B dataptr @" << _b.dataPtr << std::endl;
  gzerr << "COMPARING @" << &_a << " with @" << &_b  << ": " << r << std::endl;
  return r;
}


/////////////////////////////////////////////////
const RelativePose FrameGraph::Invalid() const
{
  return this->invalid;
}


/////////////////////////////////////////////////
bool FrameGraph::ParentFrame(const std::string &_frame,
            std::string &_parent, bool canonical) const
{

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  std::cerr << "FrameGraph::Parent not implemented " << std::endl;
  return false;
}

RelativePose::RelativePose()
  :dataPtr(NULL)  // this is the invalid frame
{
 gzerr << " RelativePose::RelativePose()\n" ;
}

Pose3d RelativePose::Compute() const
{
  Pose3d p;
  std::cerr << "DOES NOT COMPUTE!!!" << std::endl;
  return p;
}


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


/*
/////////////////////////////////////////////////
FramePrivate* FrameGraph::FrameFromAbsolutePath(
                                               const std::string &_path) const
{

  // Is it a good name?
  Tokens path(_path);
  if (!path.IsFull())
  {
    gzerr << "Error: frame path \"" << _name
      << "\" is not a valid, fully qualified path"
      << std::endl;
    return false;
  }

  // we know the path is full and thus it starts with the world frame
  FramePrivate *srcFrame = this->world;
  for (size_t i=1; i < path.Elems().size() -1; ++i)
  {
    auto &children = srcFrame->children;
    std::string e = path.Elems()[i];
    gzerr << "] *** " << i << ": " << e << std::endl;
    if(children.find(e) == children.end())
    {
      gzerr << "Missing frame element: \"" << e
        << "\" in path \"" << _name << "\""  << std::endl;
      return NULL;
    }
  }
  return srcFrame;
}


/////////////////////////////////////////////////
FramePrivate* FrameGraph::FrameFromRelativePath( FramePrivate *_frame,
                                               const std::string &_path) const
{
  return NULL;
}
*/
