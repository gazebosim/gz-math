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

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
Frame::Frame()
{
}

/////////////////////////////////////////////////
Frame::Frame(const std::string &_name, const Pose3d &_pose)
  :name(_name), pose(_pose)
{
}

/////////////////////////////////////////////////
Frame::~Frame()
{
}

/////////////////////////////////////////////////
std::string Frame::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
void Frame::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
Pose3d Frame::Pose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
void Frame::SetPose(const Pose3d &_pose)
{
  this->pose = _pose;
}
