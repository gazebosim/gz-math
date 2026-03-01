/*
 * Copyright (C) 2026 Jiayi Cai
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
#ifndef GZ_MATH_SWERVEDRIVEODOMETRY_HH_
#define GZ_MATH_SWERVEDRIVEODOMETRY_HH_

#include <chrono>
#include <gz/math/Angle.hh>
#include <gz/math/Export.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz::math
{
  inline namespace GZ_MATH_VERSION_NAMESPACE {
  /// \class SwerveDriveOdometry SwerveDriveOdometry.hh
  /// gz/math/SwerveDriveOdometry.hh
  ///
  /// \brief Computes odometry values based on a set of kinematic
  /// properties and wheel speeds for a swerve-drive vehicle.
  ///
  /// A vehicle with a heading of zero degrees has a local
  /// reference frame according to the diagram below.
  ///
  ///       Y
  ///       ^
  ///       |
  ///       |
  ///       O--->X(forward)
  class GZ_MATH_VISIBLE SwerveDriveOdometry
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    /// \brief Constructor.
    /// \param[in] _windowSize Rolling window size used to compute the
    /// velocity mean
    public: explicit SwerveDriveOdometry(size_t _windowSize = 10);

    /// \brief Initialize the odometry
    /// \param[in] _time Current time.
    public: void Init(const clock::time_point &_time);

    /// \brief Get whether Init has been called.
    /// \return True if Init has been called, false otherwise.
    public: bool Initialized() const;

    /// \brief Updates the odometry class with latest wheels and
    /// steerings position
    /// \param[in] _frontLeftPos Left wheel position in radians.
    /// \param[in] _frontRightPos Right wheel position in radians.
    /// \param[in] _backLeftPos Left wheel position in radians.
    /// \param[in] _backRightPos Right wheel position in radians.
    /// \param[in] _frontLeftSteeringPos Left steering position in radians.
    /// \param[in] _frontRightSteeringPos Right steering position in radians.
    /// \param[in] _backLeftSteeringPos Left steering position in radians.
    /// \param[in] _backRightSteeringPos Right steering position in radians.
    /// \param[in] _time Current time point.
    /// \return True if the odometry is actually updated.
    public: bool Update(
      const Angle &_frontLeftPos, const Angle &_frontRightPos,
      const Angle &_backLeftPos, const Angle &_backRightPos,
      const Angle &_frontLeftSteeringPos, const Angle &_frontRightSteeringPos,
      const Angle &_backLeftSteeringPos, const Angle &_backRightSteeringPos,
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

    /// \brief Get the lateral velocity.
    /// \return The lateral velocity in meter/second.
    public: double LateralVelocity() const;

    /// \brief Get the angular velocity.
    /// \return The angular velocity in radian/second.
    public: const Angle &AngularVelocity() const;

    /// \brief Set the wheel parameters including the radius and separation.
    /// \param[in] _wheelSeparation Distance between left and right wheels.
    /// \param[in] _wheelBase Distance between front and back wheels.
    /// \param[in] _wheelRadius Radius of the wheel.
    public: void SetWheelParams(double _wheelSeparation, double _wheelBase,
                    double _wheelRadius);
    /// \brief Set the velocity rolling window size.
    /// \param[in] _size The Velocity rolling window size.
    public: void SetVelocityRollingWindowSize(size_t _size);

    /// \brief Get the wheel separation
    /// \return Distance between left and right wheels in meters.
    public: double WheelSeparation() const;

    /// \brief Get the wheel base
    /// \return Distance between front and back wheels in meters.
    public: double WheelBase() const;

    /// \brief Get the wheel radius
    /// \return Wheel radius in meters.
    public: double WheelRadius() const;

    /// \brief Private data pointer.
    GZ_UTILS_IMPL_PTR(dataPtr)
  };
  }  // namespace GZ_MATH_VERSION_NAMESPACE
}  // namespace gz::math
#endif  // GZ_MATH_SWERVEDRIVEODOMETRY_HH_
