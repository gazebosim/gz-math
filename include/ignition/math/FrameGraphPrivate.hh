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
#include <array>
#include <map>
#include <ignition/math/Pose3.hh>

#include <ignition/math/FrameGraph.hh>

namespace ignition
{
  namespace math
  {
    /// \brief A utility class to parse a path like "/world/sphere/center" into
    /// its components. Paths can be absolute or relative (i.e "../left/right")
    class PathPrivate
    {
      /// \brief Constructor
      public: PathPrivate(const std::string &_path);

      /// \brief Access to the path elements
      /// \return A vector of path elements
      public: const std::vector<std::string> & Elems() const;

      /// \brief Access the first element of the path
      /// \return The first path element
      public: const std::string &Root() const;

      /// \brief Access the last element of the path
      public: const std::string &Leaf() const;

      /// \brief Access the path
      /// \return The path in a single string (same as used in constructor)
      public: const std::string &Path() const;

      /// \brief Checks if the path is full (starts with "/world")
      /// \return True if it is a full path
      public: bool IsFull() const;

      /// \brief Checks if a string is a valid path element. It muse constain
      /// no space, punctuation or special characters
      /// \return True if the name is valid
      public: static bool CheckName(const std::string &_name);

      /// \brief Outputs the path elements to std::cout. Debugging method.
      private: void Dump() const;

      /// \brief The path string to be parsed
      private: std::string path;

      /// \brief The elements of the path
      private: std::vector<std::string> pathElems;
    };

    /// \internal
    /// \brief Private Frame data class.
    class FramePrivate
    {
      /// \brief Constructor
      /// \param[in] _name The frame's short name
      /// \param[in] _pose The frame's offset from it's parent
      /// \param[in] _parentFrame The frame's parent
      /// \param[in] _mutex The lock for the frameGraph
      public: FramePrivate(const std::string &_name,
                           const Pose3d& _pose,
                           Frame *_parentFrame);

      /// \brief Destructor
      public: ~FramePrivate();

      /// \brief Name
      public: std::string name;

      /// \brief Pose (offset from the parent frame)
      public: Pose3d pose;

      /// \brief this is a direct pointer to the parent
      /// frame, that speeds up lookup.
      public: Frame *parentFrame;

      /// Children frames, with name
      /// THIS IS AN OPPORTUNITY FOR SHARED_PTR
      /// http://stackoverflow.com/questions/20754370/about-race-condition-of-weak-ptr
      public: std::map<std::string, const Frame*> children;
    };

    /// \internal
    /// \brief Relative pose between 2 Frames in a FrameGraph.
    /// It keeps 2 lists of poses
    class RelativePosePrivate
    {
      /// \brief Constructor
      public: RelativePosePrivate();

      /// \brief Destructor
      public: ~RelativePosePrivate() = default;

      /// \brief List of frames to apply in the up direction.
      /// these are the frames from from the src frame towards
      /// the common ancestor (possibly the /world frame)
      public: std::vector<const Frame *> up;

      /// \brief List of frames to apply in the down direction,
      /// towards the
      public: std::vector<const Frame *> down;
    };

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
      public: const Frame& FrameFromAbsolutePath(
                                               const PathPrivate& _path) const;

      /// \brief non const version of FrameFromAbsolutePath
      public: Frame& FrameFromAbsolutePath(const PathPrivate& _path);

      /// NOTE: _frame SHOULD BE A REFERENCE

      /// \brief Returns a reference to a Frame, given a start Frame and
      /// a relative path.
      /// \param[in] _frame The starting Frame
      /// \param[in] _relPath The relative path from the starting Frame
      /// \return The Frame's reference
      public: const Frame& FrameFromRelativePath(const Frame *_frame,
                                           const  PathPrivate& _relPath) const;

      /// \brief The world frame, root of all frames
      public: Frame world;

      /// \brief Mutex for concurrent access to the graph
      public: mutable std::mutex mutex;
    };
  }
}

#endif
