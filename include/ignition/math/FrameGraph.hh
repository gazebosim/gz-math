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

    /// \brief Exception class for the FrameGraph and related classes.
    class IGNITION_VISIBLE FrameException : public std::runtime_error
    {
      /// \b brief Constructor with error message. Most common error is Trying
      //// to access missing paths..
      public: FrameException(const std::string &msg) : std::runtime_error(msg)
              {}
    };

    /// \brief Frame class. A frame has an offset (a Pose3d) and a parent
    /// frame. Frames are composed inside the FrameGraph class.
    class IGNITION_VISIBLE Frame
    {
      friend class FrameGraph;
      friend class FrameGraphPrivate;
      friend class RelativePose;

      /// \brief Create a new Frame to be added
      public: Frame(const std::string &_name,
                    const Pose3d &_pose,
                    Frame *_parentFramei);

      /// \brief Destructor
      public: ~Frame();

      /// \brief Copy constructor
      public: Frame(const Frame &_other);

      /// \brief assignment operator
      public: Frame& operator = (const Frame &_other);

      /// \brief Name getter
      /// \return The name of the Frame (short name, not a path)
      public: const std::string& Name() const;

      /// OMG!!! this is sooo unsafe!
      private: const Frame* ParentFrame() const;

      /// \brief Pimp my class :-(
      private: FramePrivate *dataPtr;
    };

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
      public: RelativePose& operator = (const RelativePose &_other);

      /// \brief destructor
      public : virtual ~RelativePose();

      /// \brief private constructor.
      /// \param[in] The source frame (must be a full path)
      /// \param[in] The destination frame (can be relative)
      private: RelativePose(const Frame *_srcFrame, const Frame *_dstFrame);

      /// private data
      private: RelativePosePrivate *dataPtr;
    };

    /// \brief A collection of Frames, and their relative poses
    class IGNITION_VISIBLE FrameGraph
    {
      /// OMG! there is no way to remove a Frame!!!

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

      /// \brief Computes the relative pose between 2 frames, using
      /// a RelativePose Instance
      /// \param[in] _relativePose
      public: Pose3d Pose(const RelativePose &_relativePose) const;


      /// \brief Gets the pose of a Frame (from it's parent frame)
      /// param[in] _frame The frame reference
      /// \return The pose of the frame
      public: Pose3d Pose(const Frame &_frame) const;

      /// \brief Sets the pose of a frame (from it's parent frame)
      public: void Pose(const Frame &_frame, const Pose3d &_p);

      /// \brief This method returns a Relative pose.
      /// \param[in] _srcPath The source frame path (must be absolute)
      /// \param[in] _dstPath The destination frame (can be relative)
      public: RelativePose RelativePoses(const std::string &_srcPath,
                                         const std::string &_dstPath) const;

      /// \brief Returns a reference to a frame instance
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
