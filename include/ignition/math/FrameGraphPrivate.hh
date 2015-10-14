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

#include <mutex>

#include <ignition/math/Types.hh>
#include <ignition/math/FrameGraph.hh>

#include "PathPrivate.hh"
#include "FramePrivate.hh"

namespace ignition
{
  namespace math
  {
    /// \internal
    /// \brief Private data for the Frustum class
    class FrameGraphPrivate
    {
      /// \brief Constructor
      public: FrameGraphPrivate();

      /// \brief Destructor
      public: ~FrameGraphPrivate();

      /// \brief Given an absolute path (starts with "/world"), This method
      /// returns a reference to the Frame
      /// \param[in] _path The path to the frame
      /// \return The reference to the Frame element if it exists
      public: FrameWeakPtr FrameFromAbsolutePath(
                  const PathPrivate &_path) const;

      /// \brief Returns a reference to a Frame, given a start Frame and
      /// a relative path.
      /// \param[in] _frame The starting Frame
      /// \param[in] _relPath The relative path from the starting Frame
      /// \return The Frame's reference
      public: FrameWeakPtr FrameFromRelativePath(const FrameWeakPtr &_frame,
                  const PathPrivate &_relPath) const;

      /// \brief The world frame, root of all frames
      public: FramePtr world;

      /// \brief Mutex for concurrent access to the graph
      public: mutable std::mutex mutex;
    };
  }
}
#endif
