/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <set>

#include "ignition/math/Frame.hh"
#include "ignition/math/FrameGraph.hh"

using namespace ignition;
using namespace math;
/////////////////////////////////////////////////
Pose3d FrameGraph::Transform(const std::string &_start, const std::string &_end)
{
  Pose3d result;

  std::list<Frame> path;
  this->Path(_start, _end, path);
  for (auto const &frame : path)
  {
    result += frame.Pose();
  }

  return result;
}
