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
#ifndef _IGNITION_FRAMEGRAPH_PRIVATE_HH_
#define _IGNITION_FRAMEGRAPH_PRIVATE_HH_

#include <array>
#include <map>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {

    class Frame
    {
      public: Frame(const std::string &_name,
                    const Pose3d& _pose,
                    Frame *_parent);

      public: std::string name;
      public: Pose3d pose;
      public: Frame *parent;

    };

    /// \internal
    /// \brief Private data for the Frustum class
    class FrameGraphPrivate
    {
      /// \brief Constructor
      public: FrameGraphPrivate();
      public: Frame worldFrame;
      public: std::map<std::string, Frame*> frames;
    };
  }
}

#endif
