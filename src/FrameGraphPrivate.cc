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
  : root(new Frame("/", Pose3d(), FrameWeakPtr()))
{
}

/////////////////////////////////////////////////
FrameWeakPtr FrameGraphPrivate::FrameFromAbsolutePath(
    const PathPrivate &_path) const
{
  // Is it a good path?
  if (!_path.IsAbsolute())
  {
    throw FrameException("Frame path '" + _path.Path() +
        "' is not an absolute, fully qualified path");
  }

  // we know the path is full and thus it starts with the root frame
  // const Frame *srcFrame = &this->root;
  auto srcFrame = this->root;
  for (size_t i = 0; i < _path.Elems().size(); ++i)
  {
    std::string e = _path.Elems()[i];

    if (e == "..")
    {
      if (srcFrame->ParentFrame().expired())
      {
        throw FrameException("Path '" + _path.Path() + "' is not absolute");
      }
      if (auto frame = srcFrame->ParentFrame().lock())
      {
        srcFrame = frame;
      }
      continue;
    }
    if (auto frame = srcFrame->Child(e).lock())
    {
      srcFrame = frame;
    }
    else
    {
      throw FrameException("Missing frame element: '" + e +
          "' in path '" + _path.Path() + "'");
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
      auto p = frame->ParentFrame().lock();
      frame = p;
      continue;
    }
    // follow child element e
    auto child = frame->Child(e);
    if (!(frame = child.lock()))
    {
      throw FrameException("Error: path '" + _path.Path() +
          "' contains unknown element '" + e + "'");
    }
  }
  return frame;
}
