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
Frame::~Frame()
{
  delete this->dataPtr;
}

/////////////////////////////////////////////////
std::string Frame::Name() const
{
  return this->dataPtr->name;
}

