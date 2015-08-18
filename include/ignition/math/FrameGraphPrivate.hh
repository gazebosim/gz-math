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
    class FramePrivate
    {
      public: FramePrivate(const std::string &_name,
                        const Pose3d& _pose,
                        FramePrivate*_parentFrame);
      public: std::string name;
      public: Pose3d pose;
      // this is a direct pointer to the parent
      // frame, that speeds up lookup.
      public: FramePrivate *parentFrame;

      public: std::map<std::string, FramePrivate*> chidren;
    };

    class RelativePosePrivate
    {
      public: RelativePosePrivate(FramePrivate *_src, FramePrivate *_dst);
      public: Pose3d Compute() const;

      private: std::vector<FramePrivate *> up;
      private: std::vector<FramePrivate *> down;
    };

    /// \internal
    /// \brief Private data for the Frustum class
    class FrameGraphPrivate
    {
      /// \brief Constructor
      public: FrameGraphPrivate();
      public: ~FrameGraphPrivate();

      public: FramePrivate world;
      // public: RelativePosePrivate invalid;
      // public: Frame worldFrame;
      // public: Frame unknownFrame;
    };
  }
}

#endif
