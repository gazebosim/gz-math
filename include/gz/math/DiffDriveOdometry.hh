/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_MATH_DIFFDRIVEODOMETRY_HH_
#define GZ_MATH_DIFFDRIVEODOMETRY_HH_

#include <chrono>
#include <gz/math/Angle.hh>
#include <gz/math/Export.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::math
{
  // Use a steady clock
  using clock = std::chrono::steady_clock;

  // Inline bracket to help doxygen filtering.
  inline namespace GZ_MATH_VERSION_NAMESPACE {
  /** \class DiffDriveOdometry DiffDriveOdometry.hh \
   * gz/math/DiffDriveOdometry.hh
   **/
  /// \brief Computes odometry values based on a set of kinematic
  /// properties and wheel speeds for a diff-drive vehicle.
  ///
  /// A vehicle with a heading of zero degrees has a local
  /// reference frame according to the diagram below.
  ///
  ///       Y
  ///       ^
  ///       |
  ///       |
  ///       O--->X(forward)
  ///
  /// Rotating the right wheel while keeping the left wheel fixed will cause
  /// the vehicle to rotate counter-clockwise. For example (excuse the
  /// lack of precision with ASCII art):
  ///
  ///     Y     X(forward)
  ///     ^     ^
  ///      \   /
  ///       \ /
  ///        O
  ///
  /// **Example Usage**
  ///
  /// \code{.cpp}
  /// gz::math::DiffDriveOdometry odom;
  /// odom.SetWheelParams(2.0, 0.5, 0.5);
  /// odom.Init(std::chrono::steady_clock::now());
  ///
  /// // ... Some time later
  ///
  /// // Both wheels have rotated the same amount
  /// odom.Update(GZ_DTOR(2), GZ_DTOR(2), std::chrono::steady_clock::now());
  ///
  /// // ... Some time later
  ///
  /// // The left wheel has rotated, the right wheel did not rotate
  /// odom.Update(GZ_DTOR(4), GZ_DTOR(2), std::chrono::steady_clock::now());
  /// \endcode
  class GZ_MATH_VISIBLE DiffDriveOdometry
  {
    /// \brief Constructor.
    /// \param[in] _windowSize Rolling window size used to compute the
    /// velocity mean
    public: explicit DiffDriveOdometry(size_t _windowSize = 10);

    /// \brief Initialize the odometry
    /// \param[in] _time Current time.
    public: void Init(const clock::time_point &_time);

    /// \brief Get whether Init has been called.
    /// \return True if Init has been called, false otherwise.
    public: bool Initialized() const;

    /// \brief Updates the odometry class with latest wheels and
    /// steerings position
    /// \param[in] _leftPos Left wheel position in radians.
    /// \param[in] _rightPos Right wheel position in radians.
    /// \param[in] _time Current time point.
    /// \return True if the odometry is actually updated.
    public: bool Update(const Angle &_leftPos, const Angle &_rightPos,
                        const clock::time_point &_time);

    /// \brief Get the heading.
    /// \return The heading in radians.
    public: const Angle &Heading() const;

    /// \brief Get the X position.
    ///  \return The X position in meters
    public: double X() const;

    /// \brief Get the Y position.
    /// \return The Y position in meters.
    public: double Y() const;

    /// \brief Get the linear velocity.
    /// \return The linear velocity in meter/second.
    public: double LinearVelocity() const;

    /// \brief Get the angular velocity.
    /// \return The angular velocity in radian/second.
    public: const Angle &AngularVelocity() const;

    /// \brief Set the wheel parameters including the radius and separation.
    /// \param[in] _wheelSeparation Distance between left and right wheels.
    /// \param[in] _leftWheelRadius Radius of the left wheel.
    /// \param[in] _rightWheelRadius Radius of the right wheel.
    public: void SetWheelParams(double _wheelSeparation,
                    double _leftWheelRadius,
                    double _rightWheelRadius);

    /// \brief Set the velocity rolling window size.
    /// \param[in] _size The Velocity rolling window size.
    public: void SetVelocityRollingWindowSize(size_t _size);

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }  // namespace GZ_MATH_VERSION_NAMESPACE
}  // namespace gz::math
#endif  // GZ_MATH_DIFFDRIVEODOMETRY_HH_
