/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GZ_MATH_MOVINGWINDOWFILTER_HH_
#define GZ_MATH_MOVINGWINDOWFILTER_HH_

#include <memory>
#include <vector>

#include <gz/math/config.hh>
#include <gz/math/Export.hh>
#include <gz/math/Vector3.hh>

#include <gz/utils/SuppressWarning.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    /// \brief Base class for MovingWindowFilter. This replaces the
    /// version of MovingWindowFilter in the Gazebo Common library.
    ///
    /// The default window size is 4.
    template< typename T>
    class GZ_MATH_VISIBLE MovingWindowFilter
    {
      /// \brief Constructor
      public: MovingWindowFilter(unsigned int _windowSize = 4);

      /// \brief Destructor
      public: virtual ~MovingWindowFilter() = default;

      /// \brief Update value of filter
      /// \param[in] _val new raw value
      public: void Update(const T _val);

      /// \brief Set window size
      /// \param[in] _n new desired window size
      public: void SetWindowSize(const unsigned int _n);

      /// \brief Get the window size.
      /// \return The size of the moving window.
      public: unsigned int WindowSize() const;

      /// \brief Get whether the window has been filled.
      /// \return True if the window has been filled.
      public: bool WindowFilled() const;

      /// \brief Get filtered result
      /// \return Latest filtered value
      public: T Value() const;

      /// \brief For moving window smoothed value
      public: unsigned int valWindowSize = 4;

      /// \brief keep track of number of elements
      public: unsigned int samples = 0;

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief buffer history of raw values
      public: std::vector<T> valHistory;

      /// \brief iterator pointing to current value in buffer
      public: typename std::vector<T>::iterator valIter;

      /// \brief keep track of running sum
      public: T sum;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };

    using MovingWindowFilteri = MovingWindowFilter<int>;
    using MovingWindowFilterf = MovingWindowFilter<float>;
    using MovingWindowFilterd = MovingWindowFilter<double>;
    using MovingWindowFilterVector3i = MovingWindowFilter<Vector3i>;
    using MovingWindowFilterVector3f = MovingWindowFilter<Vector3f>;
    using MovingWindowFilterVector3d = MovingWindowFilter<Vector3d>;
  }
  }  // namespace math
}  // namespace gz
#endif
