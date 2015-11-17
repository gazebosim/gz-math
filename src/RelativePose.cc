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
#include "ignition/math/FrameGraphPrivate.hh"
#include "ignition/math/RelativePosePrivate.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
RelativePose::RelativePose(const FrameWeakPtr &_dstFrame,
                           const FrameWeakPtr &_srcFrame)
  :dataPtr(new RelativePosePrivate)
{
  // Shared pointer locking is not thread safe. However
  // the FrameGraph mutex is locked during the
  // FrameGraph::CreateRelativePose call. This method call
  // assumes the lock is acquired.
  for (auto frame = _dstFrame.lock(); frame;
       frame = frame->ParentFrame().lock())
  {
    this->dataPtr->up.push_back(frame);
  }

  for (auto frame = _srcFrame.lock(); frame;
       frame = frame->ParentFrame().lock())
  {
    this->dataPtr->down.push_back(frame);
  }
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const RelativePose &_other)
  :dataPtr(new RelativePosePrivate())
{
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
}

/////////////////////////////////////////////////
RelativePose &RelativePose::operator=(const RelativePose &_other)
{
  this->dataPtr->up = _other.dataPtr->up;
  this->dataPtr->down = _other.dataPtr->down;
  return *this;
}

/////////////////////////////////////////////////
RelativePose::~RelativePose()
{
}

/////////////////////////////////////////////////
RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{
}

/////////////////////////////////////////////////
std::vector<FrameWeakPtr> RelativePose::Up() const
{
  return this->dataPtr->up;
}

/////////////////////////////////////////////////
std::vector<FrameWeakPtr> RelativePose::Down() const
{
  return this->dataPtr->down;
}
