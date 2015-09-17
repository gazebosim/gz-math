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

#include "ignition/math/FrameGraphPrivate.hh"

using namespace ignition;
using namespace math;

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
bool PathPrivate::CheckName(const std::string &_name)
{
  if (_name.empty())
    return false;
  if (_name.find_first_of("/!@#$%^&*\t ()\":;'.~`_+=,<>") != std::string::npos)
    return false;
  return true;
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
  if (this->path[0] != '/')
    return false;

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
    std::cout << i << "] " << e << std::endl;
    i++;
  }
}


/////////////////////////////////////////////////
FramePrivate::FramePrivate(const std::string &_name,
             const Pose3d &_pose,
             Frame *_parentFrame)
  :name(_name),
   pose(_pose),
   parentFrame(_parentFrame)
{
}

/////////////////////////////////////////////////
FramePrivate::~FramePrivate()
{
  std::cout << "FramePrivate::~FramePrivate()" << this->name << "" << std::endl;
  for (auto &kv : this->children)
  {
    delete kv.second;
  }
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
const Frame& FrameGraphPrivate::FrameFromAbsolutePath(
                                               const PathPrivate &_path) const
{
  // Is it a good path?
  if (!_path.IsFull())
  {
    std::stringstream ss;
    ss << "Frame path \"" << _path.Path()
      << "\" is not an absolute, fully qualified path";
    throw FrameException(ss.str());
  }
  // we know the path is full and thus it starts with the world frame
  const Frame *srcFrame = &this->world;
  for (size_t i=1; i < _path.Elems().size(); ++i)
  {
    auto &children = srcFrame->dataPtr->children;
    std::string e = _path.Elems()[i];
    auto it = children.find(e);
    if(it != children.end())
    {
      srcFrame = it->second;
    }
    else
    {
      std::stringstream ss;
      ss << "Missing frame element: \"" << e
        << "\" in path \"" << _path.Path() << "\"";
      throw FrameException(ss.str());
    }
  }
  return *srcFrame;
}

/////////////////////////////////////////////////
Frame& FrameGraphPrivate::FrameFromAbsolutePath(const PathPrivate &_path)
{
  // Use const casting to avoid code duplication
  const FrameGraphPrivate *me = const_cast<const FrameGraphPrivate*>(this);
  return const_cast<Frame&>(me->FrameFromAbsolutePath(_path));
}

/////////////////////////////////////////////////
const Frame& FrameGraphPrivate::FrameFromRelativePath(const Frame *_frame,
                                                const PathPrivate &_path) const
{
  if(_path.IsFull())
  {
    return this->FrameFromAbsolutePath(_path);
  }
  unsigned int i = 0;
  const Frame *frame = _frame;
  const std::vector<std::string> &elems = _path.Elems();
  for (auto e : elems)
  {
    // skip the "current" frame
    if (e == ".")
    {
      continue;
    }
    // access the "parent" frame
    if (e == "..")
    {
      if(!frame->dataPtr->parentFrame)
      {
        std::stringstream ss;
        ss << "path \"" << _path.Path() << "\" is invalid ";
        throw FrameException(ss.str());
      }
      frame = frame->dataPtr->parentFrame;
      continue;
    }
    // follow child element e
    auto it = frame->dataPtr->children.find(e);
    if (it == frame->dataPtr->children.end())
    {
      std::stringstream ss;
      ss << "Error: path \"" << _path.Path() << "\" contains unknown element \"" << e << "\"";
      throw FrameException(ss.str());
    }
    frame = it->second;
  }
  return *frame;
}

/////////////////////////////////////////////////
RelativePosePrivate::RelativePosePrivate()
{

}
