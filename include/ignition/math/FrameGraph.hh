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
#ifndef _IGNITION_FRAME_GRAPH_HH_
#define _IGNITION_FRAME_GRAPH_HH_

#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FrameGraphPrivate;

    /// \brief Mathematical representation of a frustum and related functions.
    /// This is also known as a view frustum.
    class IGNITION_VISIBLE FrameGraph
    {
      /// \brief Default constructor. With the following default values:
      public: FrameGraph();

      /// \brief Destructor
      public: virtual ~FrameGraph();

      /// \brief Adds a new frame to the graph
      /// \param[in]
      public: bool  AddFrame( const std::string &_name,
                              const Pose3d &_pose,
                              const std::string &_parent = "world");

      /// \brief Find a relative pose between 2 frames
      public: bool  Pose(const std::string &_srcFrame,
                         const std::string &_dstFrame,
                         Pose3d &_result) const;

      /// \brief Copy Constructor (not allowed)
      /// \param[in] _copy FrameGraph to copy.
      private: FrameGraph(const FrameGraph &_copy);

      /// \brief Assignment operator (not allowed)
      /// \param[in] _assign FrameGraph to get values from
      private: FrameGraph &operator=(const FrameGraph &_assign);

      /// \internal
      /// \brief Private data pointer
      private: FrameGraphPrivate *dataPtr;
    };
  }
}

#endif
