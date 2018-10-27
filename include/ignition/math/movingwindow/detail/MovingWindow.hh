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
#ifndef IGNITION_MATH_DETAIL_MOVING_WINDOW_HH_
#define IGNITION_MATH_DETAIL_MOVING_WINDOW_HH_

#include <algorithm>
#include <unordered_map>
#include <ignition/math/movingwindow/MovingWindow.hh>

namespace ignition
{
namespace math
{
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace movingwindow
{

template <class WindowShape, class EntityShape>
class MovingWindowPrivate
{
  /// \brief Constructor
  public: MovingWindowPrivate(const WindowShape &_winShape, double _hysteresis,
                              const Pose3d &_pose);

  /// \brief Holds shape and pose of the window
  public: WindowInfo<WindowShape> winInfo;

  public: std::unordered_map<std::size_t, 
            ShapeInfo<EntityShape>> entities;
};

template <class WindowShape, class EntityShape>
MovingWindowPrivate<WindowShape, EntityShape>::MovingWindowPrivate(
    const WindowShape &_winShape, double _hysteresis, const Pose3d &_pose)
    : winInfo{_winShape, _hysteresis, _pose}
{
  // do nothing
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
MovingWindow<WindowPolicy, EntityShape>::MovingWindow(const WindowShape &_shape,
                                                      double _hysteresis,
                                                      const Pose3d &_pose)
    : dataPtr(std::make_unique<MovingWindowPrivate<WindowShape, EntityShape>>(
          _shape, _hysteresis, _pose))

{
  // do nothing
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
void MovingWindow<WindowPolicy, EntityShape>::SetWindowPose(const Pose3d &_pose)
{
  this->dataPtr->winInfo.pose = _pose;
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
bool MovingWindow<WindowPolicy, EntityShape>::RegisterEntity(
    std::size_t _id, const EntityShape &_shape, const Pose3d &_pose)
{
  // only add if the id is new
  if (this->dataPtr->entities.find(_id) == this->dataPtr->entities.end())
  {
    this->dataPtr->entities[_id] = {_shape, _pose};
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
bool MovingWindow<WindowPolicy, EntityShape>::UnregisterEntity(std::size_t _id)
{
  return (this->dataPtr->entities.erase(_id) > 0);
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
std::size_t MovingWindow<WindowPolicy, EntityShape>::EntityCount() const
{
  return this->dataPtr->entities.size();
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
bool MovingWindow<WindowPolicy, EntityShape>::SetEntityPose(std::size_t _id,
                                                           const Pose3d &_pose)
{
  if (this->dataPtr->entities.find(_id) != this->dataPtr->entities.end())
  {
    this->dataPtr->entities[_id].pose = _pose;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
template <template <class> class WindowPolicy, class EntityShape>
std::vector<EntityState> MovingWindow<WindowPolicy, EntityShape>::Check()
{
  return WindowPolicy<EntityShape>::Check(this->dataPtr->winInfo,
                                          this->dataPtr->entities);
}

}
}
}
}

#endif
