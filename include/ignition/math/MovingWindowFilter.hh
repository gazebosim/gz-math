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
#ifndef IGNITION_MATH_MOVINGWINDOWFILTER_HH_
#define IGNITION_MATH_MOVINGWINDOWFILTER_HH_

#include <ignition/math/config.hh>
#include <ignition/math/Export.hh>

#include <vector>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    /// \brief Base class for MovingWindowFilter. This replaces the
    /// version of MovingWindowFilter in the Ignition Common library.
    ///
    /// The default window size is 4.
    template< typename T>
    class MovingWindowFilter
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
      protected: unsigned int valWindowSize = 4;

      /// \brief buffer history of raw values
      protected: std::vector<T> valHistory;

      /// \brief iterator pointing to current value in buffer
      protected: typename std::vector<T>::iterator valIter;

      /// \brief keep track of running sum
      protected: T sum;

      /// \brief keep track of number of elements
      protected: unsigned int samples = 0;
    };

    typedef MovingWindowFilter<int> MovingWindowFilteri;
    typedef MovingWindowFilter<float> MovingWindowFilterf;
    typedef MovingWindowFilter<double> MovingWindowFilterd;
    typedef MovingWindowFilter<Vector3i> MovingWindowFilterVector3i;
    typedef MovingWindowFilter<Vector3f> MovingWindowFilterVector3f;
    typedef MovingWindowFilter<Vector3d> MovingWindowFilterVector3d;
    }
  }
}
#endif
