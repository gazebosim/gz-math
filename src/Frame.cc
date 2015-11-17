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

#include "ignition/math/Frame.hh"
#include "ignition/math/FramePrivate.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
Frame::Frame(const std::string &_name,
             const Pose3d &_pose,
             const FrameWeakPtr &_parentFrame)
  : dataPtr(new FramePrivate(_name, _pose, _parentFrame))
{
}

/////////////////////////////////////////////////
std::string Frame::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
const FrameChildren_M &Frame::Children() const
{
  return this->dataPtr->children;
}

/////////////////////////////////////////////////
bool Frame::HasChild(const std::string &_name) const
{
  return this->dataPtr->children.find(_name) != this->dataPtr->children.end();
}

/////////////////////////////////////////////////
bool Frame::AddChild(const std::string &_name, const Pose3d &_pose,
    const FrameWeakPtr _parent)
{
  bool result = true;

  auto it = this->dataPtr->children.find(_name);
  if (it == this->dataPtr->children.end())
  {
    this->dataPtr->children[_name].reset(
        new ignition::math::Frame(_name, _pose, _parent));
  }
  else
    result = false;

  return result;
}

/////////////////////////////////////////////////
bool Frame::DeleteChild(const std::string &_name)
{
  bool result = true;

  auto it = this->dataPtr->children.find(_name);
  if (it != this->dataPtr->children.end())
    this->dataPtr->children.erase(_name);
  else
    result = false;

  return result;
}

/////////////////////////////////////////////////
FrameWeakPtr Frame::ParentFrame() const
{
  return this->dataPtr->parentFrame;
}

/////////////////////////////////////////////////
Pose3d Frame::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Frame::SetPose(const Pose3d &_p)
{
  this->dataPtr->pose = _p;
}

/////////////////////////////////////////////////
FrameWeakPtr Frame::Child(const std::string &_name) const
{
  auto it = this->dataPtr->children.find(_name);
  if (it != this->dataPtr->children.end())
    return it->second;
  else
    return FrameWeakPtr();
}

/////////////////////////////////////////////////
void Frame::Print(std::ostream &_out) const
{
  // Print to the stream
  this->Print(_out, "");
}

/////////////////////////////////////////////////
void Frame::Print(std::ostream &_out, std::string _path) const
{
  _out << _path << this->dataPtr->name << " [" << this->dataPtr->pose << "]\n";

  _path += this->dataPtr->name;
  if (_path.back() != '/')
    _path += "/";

  // Print all children
  for (auto const &child : this->dataPtr->children)
  {
    child.second->Print(_out, _path);
  }
}
