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

#ifndef _IGNITION_FRAME_PRIVATE_HH_
#define _IGNITION_FRAME_PRIVATE_HH_

#include <string>
#include <map>

#include <ignition/math/Frame.hh>

namespace ignition
{
  namespace math
  {
    /// \internal
    /// \brief Private Frame data class.
    class FramePrivate
    {
      /// \brief Constructor
      /// \param[in] _name The frame's short name
      /// \param[in] _pose The frame's offset from it's parent
      /// \param[in] _parentFrame The frame's parent
      /// \param[in] _mutex The lock for the frameGraph
      public: FramePrivate(const std::string &_name,
                           const Pose3d &_pose,
                           const FrameWeakPtr &_parentFrame);

      /// \brief Destructor
      public: ~FramePrivate();

      /// \brief Short name of the frame
      public: std::string name;

      /// \brief Pose (offset from the parent frame)
      public: Pose3d pose;

      /// \brief this is a direct pointer to the parent
      /// frame, that speeds up lookup.
      public: FrameWeakPtr parentFrame;

      /// Children frames, with name
      /// http://stackoverflow.com/questions/20754370/about-race-condition-of-weak-ptr
      public: std::map<std::string, FramePtr> children;
    };
  }
}

#endif
