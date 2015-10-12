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
#ifndef _IGNITION_RELATIVE_POSE_PRIVATE_HH_
#define _IGNITION_RELATIVE_POSE_PRIVATE_HH_

#include <vector>

#include <ignition/math/Frame.hh>

namespace ignition
{
  namespace math
  {
    /// \internal
    /// \brief Relative pose between 2 Frames in a FrameGraph.
    /// It keeps 2 lists of poses
    class RelativePosePrivate
    {
      /// \brief Constructor
      public: RelativePosePrivate();

      /// \brief Destructor
      public: ~RelativePosePrivate() = default;

      /// \brief List of frames to apply in the up direction.
      /// these are the frames from from the src frame towards
      /// the common ancestor (possibly the /world frame)
      public: std::vector<FrameWeakPtr> up;

      /// \brief List of frames to apply in the down direction,
      /// towards the
      public: std::vector<FrameWeakPtr> down;
    };
  }
}

#endif
