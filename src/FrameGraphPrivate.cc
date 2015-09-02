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

#include "ignition/math/FrameGraphPrivate.hh"

using namespace ignition;
using namespace math;

#define gzerr std::cerr
#define gzinfo std::cout

/////////////////////////////////////////////////
PathPrivate::PathPrivate(const std::string &_s)
  :path(_s)
{
  std::stringstream ss(_s);
  std::string item;
  while (std::getline(ss, item, '/'))
  {
    if (item == "")
      continue;
    if (item == ".")
      continue;
    this->pathElems.push_back(item);
  }
}

/////////////////////////////////////////////////
const std::vector<std::string> &  PathPrivate::Elems() const
{
  return this->pathElems;
}

/////////////////////////////////////////////////
const std::string &PathPrivate::Root() const
{
  return this->pathElems[0];
}

/////////////////////////////////////////////////
const std::string & PathPrivate::Leaf() const
{
  return this->pathElems[this->pathElems.size() -1];
}

/////////////////////////////////////////////////
const std::string & PathPrivate::Path() const
{
  return this->path;
}

/////////////////////////////////////////////////
bool PathPrivate::IsFull() const
{
  // Dump();
  // is it empty?
  if (this->pathElems.size() == 0)
  {
    return false;
  }
  // does it start with world?
  if (this->pathElems[0] != "world")
  {
    return false;
  }
  for (const std::string &s : this->pathElems)
  {
    if (s == "..")
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
void  PathPrivate::Dump() const
{
  int i = 0;
  for (const std::string &e : this->pathElems)
  {
    gzerr << i << "] " << e << std::endl;
    i++;
  }
}


/////////////////////////////////////////////////
FramePrivate::FramePrivate(const std::string &_name,
                           const Pose3d& _pose,
                           const FramePrivate *_parentFrame)
  :name(_name),
   pose(_pose),
   parentFrame(_parentFrame)
{
}

/////////////////////////////////////////////////
FrameGraphPrivate::~FrameGraphPrivate()
{
  std::cout << "======== ~FrameGraphPrivate() ======================= " << std::endl;
}

/////////////////////////////////////////////////
FrameGraphPrivate::FrameGraphPrivate()
  :world("world", Pose3d(), NULL)
{
}

/////////////////////////////////////////////////
FramePrivate* FrameGraphPrivate::FrameFromAbsolutePath(
                                               const PathPrivate &_path)
{
  // Is it a good name?
  if (!_path.IsFull())
  {
    gzerr << "Error: frame path \"" << _path.Path()
      << "\" is not a valid, fully qualified path"
      << std::endl;
    return NULL;
  }

  // we know the path is full and thus it starts with the world frame
  FramePrivate *srcFrame = &this->world;
  for (size_t i=1; i < _path.Elems().size() -1; ++i)
  {
    auto &children = srcFrame->children;
    std::string e = _path.Elems()[i];
    if(children.find(e) == children.end())
    {
      gzerr << "Missing frame element: \"" << e
        << "\" in path \"" << _path.Path() << "\""  << std::endl;
      return NULL;
    }
  }
  return srcFrame;
}


/////////////////////////////////////////////////
FramePrivate* FrameGraphPrivate::FrameFromRelativePath( FramePrivate *_frame,
                                               const PathPrivate &_path)
{
  if(_path.IsFull())
  {
    return this->FrameFromAbsolutePath(_path);
  }
  return NULL;
}

/////////////////////////////////////////////////
RelativePosePrivate::RelativePosePrivate()
  :mutex(NULL)
{

}


