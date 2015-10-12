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
FramePrivate::FramePrivate(const std::string &_name,
             const Pose3d &_pose,
             const FrameWeakPtr &_parentFrame)
  :name(_name),
  pose(_pose),
  parentFrame(_parentFrame)
{
}

/////////////////////////////////////////////////
FramePrivate::~FramePrivate()
{
}

/////////////////////////////////////////////////
FrameGraphPrivate::~FrameGraphPrivate()
{
}

/////////////////////////////////////////////////
FrameGraphPrivate::FrameGraphPrivate()
  : world(new Frame("world", Pose3d(), FrameWeakPtr()))
{
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraphPrivate::FrameFromAbsolutePath(
    const PathPrivate &_path) const
{
  // Is it a good path?
  if (!_path.IsAbsolute())
  {
    std::stringstream ss;
    ss << "Frame path \"" << _path.Path()
      << "\" is not an absolute, fully qualified path";
    throw FrameException(ss.str());
  }
  // we know the path is full and thus it starts with the world frame
  // const Frame *srcFrame = &this->world;
  auto srcFrame = this->world;
  for (size_t i = 1; i < _path.Elems().size(); ++i)
  {
    const auto &children = srcFrame->dataPtr->children;
    std::string e = _path.Elems()[i];
    auto it = children.find(e);
    if (it != children.end())
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
  return srcFrame;
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraphPrivate::FrameFromRelativePath(
    const FrameWeakPtr &_frame, const PathPrivate &_path) const
{

  // path may be full
  if (_path.IsAbsolute())
  {
    return this->FrameFromAbsolutePath(_path);
  }
  auto frame = _frame.lock();
  const std::vector<std::string> &elems = _path.Elems();
  for (auto e : elems)
  {
    // access the "parent" frame
    if (e == "..")
    {
      auto p = frame->dataPtr->parentFrame.lock();
      frame = p;
      continue;
    }
    // follow child element e
    auto it = frame->dataPtr->children.find(e);
    if (it == frame->dataPtr->children.end())
    {
      std::stringstream ss;
      ss << "Error: path \"" << _path.Path()
         << "\" contains unknown element \"" << e << "\"";
      throw FrameException(ss.str());
    }
    frame = it->second;
  }
  return frame;
}

/////////////////////////////////////////////////
RelativePosePrivate::RelativePosePrivate()
{
}
