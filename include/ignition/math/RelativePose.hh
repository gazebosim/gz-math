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
      friend class FrameGraph;

      /// constructor
      public: RelativePose();

      /// \brief Copy constructor. Copies the content of the private data.
      public: RelativePose(const RelativePose &_c);

      /// \brief Assignemt operator. Copies the private data.
      public: RelativePose& operator=(const RelativePose &_other);

      /// \brief destructor
      public : virtual ~RelativePose();

      /// \brief private constructor.
      /// \param[in] The source frame (must be a full path)
      /// \param[in] The destination frame (can be relative)
      private: RelativePose(const FrameWeakPtr &_srcFrame,
          const FrameWeakPtr &_dstFrame);

      /// private data
      private: RelativePosePrivate *dataPtr;
    };
  }
}

#endif
