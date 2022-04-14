/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef IGNITION_MATH_INTERPOLATION_POINT_HH_
#define IGNITION_MATH_INTERPOLATION_POINT_HH_


namespace ignition
{
  namespace math
  {
    /// \brief Describes an interpolation point.
    template<typename T>
    struct InterpolationPoint3D
    {
      /// \brief The position of the point
      Vector3<T> position;

      /// \brief The index from which this point was retrieved.
      /// Can be used by the application calling it to index. The reason
      /// this is optional is that data may be missing from a sparse grid.
      std::optional<std::size_t> index;
    };

    /// \brief Describes an interpolation point in1d.
    template<typename T>
    struct InterpolationPoint1D
    {
      /// \brief The position of the point
      T position;

      /// \brief The index from which this point was retrieved.
      /// Can be used by the application calling it to index.
      std::size_t index;
    };
  }
}
#endif