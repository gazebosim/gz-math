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
RelativePose FrameGraph::CreateRelativePose(const std::string &_srcPath,
                                       const std::string &_dstPath) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const auto &srcFrame = this->dataPtr->FrameFromAbsolutePath(_srcPath);
  const auto &dstFrame = this->dataPtr->FrameFromRelativePath(srcFrame,
                                                              _dstPath);
  // create the relative pose object while we have the mutex lock
  RelativePose r(srcFrame, dstFrame);
  return r;
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const FrameWeakPtr &_srcFrame,
                           const FrameWeakPtr &_dstFrame)
  :dataPtr(new RelativePosePrivate())
{
  // Shared pointer locking is not thread safe. However
  // the FrameGraph mutex is locked during the
  // FrameGraph::CreateRelativePose call. This method call
  // assumes the lock is acquired.
  auto frame = _srcFrame.lock();
  while (frame && frame->dataPtr->parentFrame.lock())
  {
    this->dataPtr->up.push_back(frame);
    frame = frame->dataPtr->parentFrame.lock();
  }
  frame = _dstFrame.lock();
  while (frame && frame->dataPtr->parentFrame.lock())
  {
    this->dataPtr->down.push_back(frame);
    frame = frame->dataPtr->parentFrame.lock();
  }
}

/////////////////////////////////////////////////
RelativePose::RelativePose(const RelativePose &_other)
  :dataPtr(new RelativePosePrivate())
{
  *this->dataPtr = *_other.dataPtr;
}

/////////////////////////////////////////////////
RelativePose& RelativePose::operator=(const RelativePose &_other)
{
  if (this == &_other)
    return *this;
  *this->dataPtr = *_other.dataPtr;
  return *this;
}

/////////////////////////////////////////////////
RelativePose::~RelativePose()
{
  delete this->dataPtr;
}

/////////////////////////////////////////////////
RelativePose::RelativePose()
  :dataPtr(new RelativePosePrivate())
{
}

