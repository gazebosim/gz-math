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
#ifndef _IGNITION_RELATIVEPOSE_HH_
#define _IGNITION_RELATIVEPOSE_HH_

#include <vector>
#include <memory>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class RelativePosePrivate;

    /// \brief Holds the chain of transforms to compute a pose between
    /// two frames in a FrameGraph
    class IGNITION_VISIBLE RelativePose
    {
      /// \brief Constructor
      public: RelativePose();

      /// \brief Frame constructor.
      /// \param[in] _dstFrame The destination frame
      /// \param[in] _srcFrame The source frame
      public: RelativePose(const FrameWeakPtr &_dstFrame,
                           const FrameWeakPtr &_srcFrame);

      /// \brief destructor
      public: virtual ~RelativePose();

      /// \brief Copy constructor. Copies the content of the private data.
      /// \param[in] _c The copy
      public: RelativePose(const RelativePose &_c);

      /// \brief Assignment operator. Copies the private data.
      /// \param[in] _other The other RelativePose
      public: RelativePose &operator=(const RelativePose &_other);

      /// \brief Get the vector of frames from the destination towards
      /// the common ancestor.
      /// \return List of frames to apply in the up direction.
      public: std::vector<FrameWeakPtr> Up() const;

      /// \brief Get the vector of frames from the destination towards
      /// the common ancestor.
      /// \return List of frames to apply in the down direction.
      public: std::vector<FrameWeakPtr> Down() const;

      /// \brief Private data
      private: std::unique_ptr<RelativePosePrivate> dataPtr;
    };
  }
}
#endif
