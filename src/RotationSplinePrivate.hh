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
#ifndef GZ_MATH_ROTATIONSPLINEPRIVATE_HH_
#define GZ_MATH_ROTATIONSPLINEPRIVATE_HH_

#include <vector>
#include <gz/math/config.hh>
#include "gz/math/Quaternion.hh"

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    /// \internal
    /// \brief Private data for RotationSpline
    class RotationSplinePrivate
    {
      /// \brief Constructor
      public: RotationSplinePrivate();

      /// \brief Automatic recalculation of tangents when control points are
      /// updated
      public: bool autoCalc;

      /// \brief the control points
      public: std::vector<Quaterniond> points;

      /// \brief the tangents
      public: std::vector<Quaterniond> tangents;
    };
    }
  }
}
#endif
