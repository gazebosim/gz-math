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
#include <map>

#include <ignition/math/Types.hh>
#include <ignition/math/Pose3.hh>

namespace ignition
{
  namespace math
  {
    // Forward declaration of private data
    class FramePrivate;

    /// \type Map of child frame names to frame object.
    typedef std::map<std::string, FramePtr> FrameChildren_M;

    /// \brief Frame class. A frame has an offset (a Pose3d) and a parent
    /// frame. Frames are composed inside the FrameGraph class.
    /// The Frame class does not have a lot of public methods. The FrameGraph
    /// class acts as a Facade from which to get the Frame information, and
    /// performs the thread locking while the Frame data is accessed.
    class IGNITION_VISIBLE Frame
    {
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

      /// \brief Get the child frames.
      /// \return A map of children with their frames.
      public: const FrameChildren_M &Children() const;

      /// \brief Get a child frame.
      /// \param[in] _name Name of the child frame to get.
      /// \return Weak pointer to the child. Null if the child does not
      /// exist.
      public: FrameWeakPtr Child(const std::string &_name) const;

      /// \brief Check if a child frame exists.
      /// \param[in] _name Name of the child frame to check.
      /// \return True if the frame with the given name exists.
      public: bool HasChild(const std::string &_name) const;

      /// \brief Add a child frame.
      /// \param[in] _name Name of the child frame to delete.
      /// \param[in] _frame The child frame.
      /// \return True on success
      public: bool AddChild(const std::string &_name, const Pose3d &_pose,
                      const FrameWeakPtr _parent);

      /// \brief Delete a child frame.
      /// \param[in] _name Name of the child frame to delete.
      /// \return True if the frame with the given name exists and was
      /// deleted.
      public: bool DeleteChild(const std::string &_name);

      /// \brief Get a pointer to the parent frame.
      /// \return Pointer to the parent frame.
      public: FrameWeakPtr ParentFrame() const;

      /// \brief Get the frame's pose.
      /// \return Pose of this frame.
      public: Pose3d Pose() const;

      /// \brief Set the pose of the frame.
      /// \param[in] _p The frame's new pose
      public: void SetPose(const Pose3d &_p);

      /// \brief Stream insertion operator
      /// \param[out] _out output stream
      /// \param[in] _f Frame to output
      /// \return the stream
      public: friend std::ostream &operator<<(
                  std::ostream &_out, const ignition::math::Frame &_f)
      {
        _f.Print(_out);
        return _out;
      }

      /// \brief Print the frame graph to a stream
      /// \param[out] _out output stream
      private: void Print(std::ostream &_out) const;

      /// \brief Helper function to print the frame
      /// \param[out] _out output stream
      /// \param[in] _path Path prefix
      private: void Print(std::ostream &_out, std::string _path) const;

      /// \brief Private data
      private: std::unique_ptr<FramePrivate> dataPtr;
    };
  }
}

#endif
