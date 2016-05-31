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

const Frame ignition::math::Frame::Nan("inf", Pose3d::Nan);

namespace ignition
{
  namespace math
  {
    /// \brief Private data for the Frame class
    class FramePrivate
    {
      /// Constructor
      public: FramePrivate() {}

      /// Constructor
      /// \param[in] _name Name of the frame
      /// \param[in] _pose Pose of the frame
      public: FramePrivate(const std::string &_name, const Pose3d &_pose)
              : pose(_pose), name(_name) {}

      /// \brief Frame's pose
      public: Pose3d pose = ignition::math::Pose3d::Zero;

      /// \brief Frame's name
      public: std::string name = "";
    };
  }
}

/////////////////////////////////////////////////
Frame::Frame()
: dataPtr(new FramePrivate)
{
}

/////////////////////////////////////////////////
Frame::Frame(const std::string &_name, const Pose3d &_pose)
: dataPtr(new FramePrivate(_name, _pose))
{
}

/////////////////////////////////////////////////
Frame::Frame(const Frame &_frame)
: dataPtr(new FramePrivate)
{
  this->dataPtr->name = _frame.Name();
  this->dataPtr->pose = _frame.Pose();
}

/////////////////////////////////////////////////
Frame::~Frame()
{
}

/////////////////////////////////////////////////
std::string Frame::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void Frame::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
Pose3d Frame::Pose() const
{
  return this->dataPtr->pose;
}

/////////////////////////////////////////////////
void Frame::SetPose(const Pose3d &_pose)
{
  this->dataPtr->pose = _pose;
}

/////////////////////////////////////////////////
bool Frame::operator==(const Frame &_frame) const
{
  return this->Name() == _frame.Name() && this->Pose() == _frame.Pose();
}

/////////////////////////////////////////////////
bool Frame::operator!=(const Frame &_frame) const
{
  return !(Frame::operator==(_frame));
}
