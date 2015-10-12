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

#include <string>
#include <mutex>
#include <map>

#include <ignition/math/Types.hh>

#include "FrameException.hh"
#include "Frame.hh"
#include "RelativePose.hh"

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FrameGraphPrivate;
    /// \brief A collection of Frames, and their relative poses.
    class IGNITION_VISIBLE FrameGraph
    {
      /// \brief Default constructor. With the following default values:
      public: FrameGraph();

      /// \brief Destructor
      public: virtual ~FrameGraph();

      public: FrameGraph(const FrameGraph &_x) = delete;
 
      public: FrameGraph& operator=(const FrameGraph &_x) = delete;

      /// \brief Adds a new frame to the graph
      /// \param[in] _path The full path of the frame's parent
      /// \param[in] _name The name of the new frame
      /// \param[in] _pose The pose of the frame, relative to the parent frame
      /// \throws FrameException if one of the path is invalid.
      public: void AddFrame(const std::string &_path,
                            const std::string &_name,
                            const Pose3d &_pose);

      /// \brief Removes a frame and all its children.
      /// \param[in] _path The absolute path to the frame
      public: void DeleteFrame(const std::string &_path);

      /// \brief Computes a relative pose between 2 frames
      /// \param[in] _dst The name of the destination frame
      /// \param[in] _src The name of the source frame
      /// \return The pose of _dest in _src's frame
      public: Pose3d Pose(const std::string &_dst,
                          const std::string &_src) const;

      /// \brief Computes the relative pose between 2 frames, using
      /// a RelativePose Instance
      /// \param[in] _relativePose
      public: Pose3d Pose(const RelativePose &_relativePose) const;

      /// \brief Get the pose of a frame (relative to its parent frame)
      /// \param[in] _path The absolute path to the frame
      /// \return The local pose of the a frame
      public: Pose3d LocalPose(const std::string &_path) const;

      /// \brief Get the pose of a Frame (relative to its parent frame)
      /// param[in] _frame The frame reference
      /// \return The pose of the frame
      public: Pose3d LocalPose(const FrameWeakPtr &_frame) const;

      /// \brief Set the pose of a frame (relative to its parent frame)
      /// \param[in] _path The absolute path to the frame
      /// \return The local pose
      public: void SetLocalPose(const std::string &_path, const Pose3d &_p);

      /// \brief Set the pose of a frame (relative to its parent frame)
      /// \param[in] _frame The frame reference
      /// \return The local pose
      public: void SetLocalPose(FrameWeakPtr _frame, const Pose3d &_p);

      /// \brief This method generate a relative pose between two frames.
      /// \param[in] _srcPath The source frame path (must be absolute)
      /// \param[in] _dstPath The destination frame (can be relative)
      public: RelativePose CreateRelativePose(const std::string &_srcPath,
                  const std::string &_dstPath) const;

      /// \brief Get a reference to a frame instance
      /// \param[in] _path The absolute path to the frame
      /// \return The frame's weak pointer
      public: FrameWeakPtr FrameAccess(const std::string &_path) const;

      /// \brief Get a reference to a frame instance
      /// \param[in] _
      /// \param[in] _relativepath The relative path to the frame
      /// \return The frame's weak pointer
      public: FrameWeakPtr FrameAccess(FrameWeakPtr _frame,
                                       const std::string &_relativePath) const;

      /// \internal
      /// \brief Private data pointer
      private: FrameGraphPrivate *dataPtr;
    };
  }
}
#endif
