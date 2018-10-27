/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_MOVING_WINDOW_HH_
#define IGNITION_MATH_MOVING_WINDOW_HH_

#include <memory>
#include <vector>
#include <unordered_map>
#include <ignition/math/config.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace movingwindow
{
  // Forward declare private data
  template <class WindowShape, class EntityShape>
  class MovingWindowPrivate;

  /// brief: TODO
  struct EntityState
  {
    /// brief: TODO
    std::size_t id;

    enum State
    {
      UNINITIALIZED = 0,
      INSIDE = 1,
      OUTSIDE = 2,
    };

    /// brief: TODO
    State state = UNINITIALIZED;
  };

  /// \brief A struct that holds the shape used for the entities as well as
  /// extra parameters.
  template <class EntityShape>
  struct ShapeInfo
  {
    /// \brief The shape of the entity.
    EntityShape shape;
    /// \brief The pose of the shape in the world.
    Pose3d pose;
  };

  /// \brief A struct that holds the shape used for the window as well as
  /// extra parameters.
  template <class WindowShape>
  struct WindowInfo
  {
    /// \brief The shape of the window.
    WindowShape shape;
    /// \brief The hysteresis is taken to be a buffer around the window.
    double hysteresis;
    /// \brief The pose of the window. For certain window shapes, only the
    /// position will be used.
    Pose3d pose;
  };

  /// \brief A generic moving window implementation.
  /// \tparam WindowPolicy A type that implements \c Check(~), which determines
  /// whether entities are in the window
  /// \tparam EntityShape The shape type that represents the entity. The
  /// parameter is also passed to \a WindowPolicy.
  ///
  /// \detail This class takes \a WindowPolicy class as a template parameter.
  ///
  /// <b> Example usage</b>
  /// \code{cpp}
  /// MovingWindow<AxisAlignedBoxWindow, AxisAlignedBox> window({-10, -10, 0,
  ///                                                             10, 10, 10});
  /// \endcode
  ///
  /// <b> For window implementers</b>
  /// The policy class must be a template that accepts a template parameter for
  /// the shape of the entity.  The policy class must have also have  static
  /// function \c WindowPolicy::Check
  /// Example
  /// \code{cpp}
  ///
  ///  template <class EntityShape>
  ///  class MyPolicy
  ///  {
  ///       public: static std::vector<EntityState> Check(
  ///                 const WindowInfo<WindowShape> &_winInfo,
  ///                 const std::unordered_map<
  ///                    std::size_t, ShapeInfo<EntityShape>> &_entities)
  ///  };
  /// \endcode
  /// See \ref AxisAlignedBoxWindow for an example of how this is implemented
  ///
  ///
  template <template <class> class WindowPolicy, class EntityShape>
  class IGNITION_MATH_VISIBLE MovingWindow
  {
    public: using WindowShape =
            typename WindowPolicy<EntityShape>::WindowShape;

    /// \brief Constructor
    public: MovingWindow(const WindowShape &_winShape,
                             double _hysteresis = 0,
                             const Pose3d &_pose = Pose3d::Zero);

    /// \brief Sets the pose of the window. Note that for some windows only the
    /// position is meaningful.
    /// \param[in] _pose The pose of the window.
    public: void SetWindowPose(const Pose3d &_pose);

    /// \brief Register the given entity for later checking if in window
    /// \param[in] _id The id associated with the entity. This id is provided
    /// by the caller and will be used in the returned list of entities when
    /// Check() is called.
    /// \param[in] _shape The shape of the entity.
    /// \param[in] _pose The pose of the entity.
    /// \return True if the the entity was successfully (the _id is new)
    /// registered.
    public: bool RegisterEntity(std::size_t _id, const EntityShape &_shape,
                                const Pose3d &_pose = Pose3d::Zero);

    /// \brief Unregister the given entity from the window.
    /// \param[in] _id The id associated with the entity. This id would have
    /// been provided when calling \see{RegisterEntity}.
    /// \return True if the the entity was successfully (the _id is found)
    /// unregistered.
    public: bool UnregisterEntity(std::size_t _id);

    /// \brief Gives the number of entities registered in this window.
    /// \return number of entities registered.
    public: std::size_t EntityCount() const;

    /// \brief Sets the pose of the entity. Note that some windows only use the
    /// position of the entity and ignore its orientation.
    /// \param[in] _id The id associated with the entity.
    /// \param[in] _pose The pose of the entity.
    public: bool SetEntityPose(std::size_t _id, const Pose3d &_pose);

    /// \brief Checks if the registered entities are inside the window.
    /// see the class documentation, \ref MovingWindow, for the requirment on 
    /// the window policy for the function to work properly.
    /// \todo(addisu): About changing state. Add another function for getting
    /// objects of certain state.
    /// \returns A list of entities (identified by their ids supplied to
    /// RegisterEntity) and their state.
    public: std::vector<EntityState> Check();

    /// \brief Pointer to private data.
    private: std::unique_ptr<
            MovingWindowPrivate<WindowShape, EntityShape>> dataPtr;
  };

}
}
}
}

#include "ignition/math/movingwindow/detail/MovingWindow.hh"

#endif
