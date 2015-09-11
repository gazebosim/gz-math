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

#include <mutex>
#include <map>
#include <sstream>

#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FrameGraphPrivate;
    class RelativePosePrivate;
    class FramePrivate;


    class IGNITION_VISIBLE FrameException : public std::runtime_error
    {
      public: FrameException(const std::string &msg) : std::runtime_error(msg)
              {}
    };

    class IGNITION_VISIBLE Frame
    {
      friend class FrameGraph;
      friend class FrameGraphPrivate;

      public: Frame(const std::string &_name,
                        const Pose3d &_pose,
                        Frame *_parentFrame);
      public: ~Frame();

      public: const std::string& Name() const;

      public: const Pose3d &Pose() const;

      public: void Pose (const Pose3d &_p);

      public: const Frame* ParentFrame() const;

      private: FramePrivate *dataPtr;
    };

    class IGNITION_VISIBLE RelativePose
    {
      friend class FrameGraph;

      public: RelativePose();

      public: RelativePose(const RelativePose &_c);

      public: RelativePose& operator = (const RelativePose &_other);

      /// \brief destructor
      public : virtual ~RelativePose();

      public: Pose3d Compute() const;

      /// \brief private constructor.
      /// \param[in] The source frame must be a full path but
      /// the de
      private: RelativePose(std::mutex *mutex,
                            const Frame *_srcFrame,
                            const Frame *_dstFrame);

      private: RelativePosePrivate *dataPtr;
    };

    /// \brief A collection of Frames, and their relative poses
    class IGNITION_VISIBLE FrameGraph
    {
      /// \brief Default constructor. With the following default values:
      public: FrameGraph();

      /// \brief Destructor
      public: virtual ~FrameGraph();

      /// \brief Adds a new frame to the graph
      /// \param[in] _path The full path of the frame's parent
      /// \param[in] _name The name of the new frame
      /// \param[in] _pose The pose of the frame, relative to the parent frame
      /// \throws FrameException if one of the path is invalid.
      public: void  AddFrame( const std::string &_path,
                              const std::string &_name,
                              const Pose3d &_pose);

      /// \brief Computes a relative pose between 2 frames
      /// \param[in] _srcFrame The name of the source frame
      /// \param[in] _dstFrame The name of the destination frame
      /// \return The pose between the frames, if it exists
      public: Pose3d Pose(const std::string &_srcFrame,
                         const std::string &_dstFrame) const;


      /// \brief This method returns a Relative pose instance that is
      /// initialized to
      public: bool RelativePoses(const std::string &_srcPath,
                                const std::string &_dstPathi,
                                RelativePose &_relativePose) const;

      /// \brief Returns a reference
      public: Frame &FrameAccess(const std::string &_path) const;

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
