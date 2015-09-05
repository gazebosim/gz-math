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
void PathPrivate::PopLeaf()
{
  if (this->pathElems.size() >1)
  {
    this->pathElems.pop_back();
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
}

/////////////////////////////////////////////////
FrameGraphPrivate::FrameGraphPrivate()
  :world("world", Pose3d(), NULL)
{
}

/////////////////////////////////////////////////
const FramePrivate* FrameGraphPrivate::FrameFromAbsolutePath(
                                               const PathPrivate &_path) const
{
  // Is it a good path?
  if (!_path.IsFull())
  {
    gzerr << "Error: frame path \"" << _path.Path()
      << "\" is not a valid, fully qualified path"
      << std::endl;
    return NULL;
  }

  // we know the path is full and thus it starts with the world frame
  const FramePrivate *srcFrame = &this->world;
  for (size_t i=1; i < _path.Elems().size(); ++i)
  {
    auto &children = srcFrame->children;
    std::string e = _path.Elems()[i];
    auto it = children.find(e);
    if(it != children.end())
    {
      srcFrame = it->second;
    }
    else
    {
      gzerr << "Missing frame element: \"" << e
        << "\" in path \"" << _path.Path() << "\""  << std::endl;
      return NULL;
    }
  }
  return srcFrame;
}

/////////////////////////////////////////////////
FramePrivate* FrameGraphPrivate::FrameFromAbsolutePath(
                                               const PathPrivate &_path)
{
  const FrameGraphPrivate *me = const_cast<const FrameGraphPrivate*>(this);
  auto frame = me->FrameFromAbsolutePath(_path);
  return const_cast<FramePrivate*>(frame);
}

/////////////////////////////////////////////////
const FramePrivate* FrameGraphPrivate::FrameFromRelativePath(
                                                    const FramePrivate *_frame,
                                                    const PathPrivate &_path) const
{
  if(_path.IsFull())
  {
    return this->FrameFromAbsolutePath(_path);
  }
  unsigned int i = 0;
  const FramePrivate *frame = _frame;
  const std::vector<std::string> &elems = _path.Elems();
  for (auto e : elems)
  {
gzerr << "*** *** ***" << _frame->name << " [" << e  << "?]" << std::endl;
    if (e == ".")
    {
      continue;
    }
    if (e == "..")
    {
      if(!frame->parentFrame)
      {
        gzerr << "Error: path \"" << _path.Path() << "\" is invalid" << std::endl;
        return NULL;
      }
      frame = frame->parentFrame;
      continue;
    }
    auto it = frame->children.find(e);
    if (it == frame->children.end())
    {
      gzerr << "Error: path \"" << _path.Path() << "\" contains unknown element \"" << e << "\"" << std::endl;
    }
    frame = it->second;
  }
  return frame;
}

/////////////////////////////////////////////////
RelativePosePrivate::RelativePosePrivate()
  :mutex(NULL)
{

}


