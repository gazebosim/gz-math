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
#ifndef _IGNITION_FRAME_HH_
#define _IGNITION_FRAME_HH_

#include <memory>
#include <string>

#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FramePrivate;
    // Forward declaration for pointers
    class Frame;
    // Pointer types
    using FramePtr = std::shared_ptr<Frame>;
    using FrameWeakPtr = std::weak_ptr<Frame>;
    using FramePrivatePtr = std::unique_ptr<FramePrivate>;

    /// \brief Frame class. A frame has an offset (a Pose3d) and a parent
    /// frame. Frames are composed inside the FrameGraph class.
    /// The Frame class does not have a lot of public methods. The FrameGraph
    /// class acts as a Facade from which get the Frame information, and
    /// performs the thread locking while the Frame data is accessed.
    class IGNITION_VISIBLE Frame
    {
      // adding and deleting Frames
      friend class FrameGraph;
      // path calculations
      friend class FrameGraphPrivate;
      // access Frame's parents
      friend class RelativePose;

      /// \brief Create a new Frame to be added
      /// \param[in] _name Short name of the frame
      /// \param[in] _pose Pose relative to the parent frame
      /// \param[in] _parentFrame The parent frame
      public: Frame(const std::string &_name,
                    const Pose3d &_pose,
                    const FrameWeakPtr &_parentFrame);

      /// \brief Name getter
      /// \return The name of the Frame (short name, not a path)
      public: std::string Name() const;

      /// \brief Private data
      private: FramePrivatePtr dataPtr;
    };
  }
}

#endif
