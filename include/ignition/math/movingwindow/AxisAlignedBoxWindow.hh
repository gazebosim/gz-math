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

#ifndef IGNITION_MATH_MOVINGWINDOW_AXIS_ALIGNED_BOX_WINDOW_HH_
#define IGNITION_MATH_MOVINGWINDOW_AXIS_ALIGNED_BOX_WINDOW_HH_

#include <ignition/math/movingwindow/MovingWindow.hh>

namespace ignition
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace movingwindow
{
  /// \brief A base window policy based on AxisAlignedBox shapes.
  template <class EntityShape>
  class AxisAlignedBoxWindow;

  /// \brief Template specialization for when EntityShape is an AxisAlignedBox
  template <>
  class AxisAlignedBoxWindow<AxisAlignedBox>
  {
    public: using WindowShape = AxisAlignedBox;
    public: using EntityShape = AxisAlignedBox;

    /// brief: TODO
    public: static std::vector<EntityState> Check(
              const WindowInfo<WindowShape> &_winInfo,
              const std::unordered_map<std::size_t, ShapeInfo<EntityShape>>
                  &_entities)
        {
          std::vector<EntityState> output;

          for (const auto &[id, shapeInfo] : _entities)
          {
            // We consider intersection to be inside the window
            bool inside = _winInfo.shape.Intersects(shapeInfo.shape +
                                                    shapeInfo.pose.Pos());
            EntityState::State state =
                inside ? EntityState::INSIDE : EntityState::OUTSIDE;
            output.push_back(EntityState{id, state});
          }
          return output;
        }
  };

  template <class EntityShape>
  using AxisAlignedBoxMovingWindow =
      MovingWindow<AxisAlignedBoxWindow, EntityShape>;
}
}
}
}

#endif
