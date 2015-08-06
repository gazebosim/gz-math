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
Frame::Frame(const std::string &_name,
             const std::string &_parent,
                    const Pose3d& _pose,
                    Frame *_parentFrame)
  :name(_name), parent(_parent), pose(_pose), parentFrame(_parentFrame)
{
}

/////////////////////////////////////////////////
FrameGraphPrivate::FrameGraphPrivate()
  :worldFrame("world", "", Pose3d(), NULL),
   unknownFrame("unknown", "", Pose3d(), NULL)
{

  this->frames["world"] =  &this->worldFrame;
  this->frames["unknown"] = &this->unknownFrame;
}


