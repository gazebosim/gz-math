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
#ifndef IGNITION_MATH_MECANUMDRIVEODOMETRY_HH_
#define IGNITION_MATH_MECANUMDRIVEODOMETRY_HH_

#include <chrono>
#include <memory>
#include <ignition/math/Angle.hh>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Use a steady clock
    using clock = std::chrono::steady_clock;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declarations.
    class MecanumDriveOdometryPrivate;

    /** \class MecanumDriveOdometry MecanumDriveOdometry.hh \
     * ignition/math/MecanumDriveOdometry.hh
     **/
    /// \brief Computes odometry values based on a set of kinematic
    /// properties and wheel speeds for a Mecanum-drive vehicle.
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


    class IGNITION_MATH_VISIBLE MecanumDriveOdometry
    {
      /// \brief Constructor.
      /// \param[in] _windowSize Rolling window size used to compute the
      /// velocity mean
      public: explicit MecanumDriveOdometry(size_t _windowSize = 10);

      /// \brief Destructor.
      public: ~MecanumDriveOdometry();

      /// \brief Initialize the odometry
      /// \param[in] _time Current time.
      public: void Init(const clock::time_point &_time);

      /// \brief Get whether Init has been called.
      /// \return True if Init has been called, false otherwise.
      public: bool Initialized() const;

      /// \brief Updates the odometry class with latest wheels and
      /// steerings position
      /// \param[in] _frontLeftPos Left wheel position in radians.
      /// \param[in] _frontRightPos Right wheel postion in radians.
      /// \param[in] _backLeftPos Left wheel position in radians.
      /// \param[in] _backRightPos Right wheel postion in radians.
      /// \param[in] _time Current time point.
      /// \return True if the odometry is actually updated.
      public: bool Update(const Angle &_frontLeftPos, const Angle &_frontRightPos, const Angle &_backLeftPos, const Angle &_backRightPos,
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
      /// \param[in] _leftWheelRadius Radius of the left wheel.
      /// \param[in] _rightWheelRadius Radius of the right wheel.
      public: void SetWheelParams(double _wheelSeparation, double _wheelBase,
                      double _leftWheelRadius, double _rightWheelRadius);

      /// \brief Set the velocity rolling window size.
      /// \param[in] _size The Velocity rolling window size.
      public: void SetVelocityRollingWindowSize(size_t _size);

#ifdef _WIN32
// Disable warning C4251 which is triggered by
// std::unique_ptr
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
      /// \brief Private data pointer.
      private: std::unique_ptr<MecanumDriveOdometryPrivate> dataPtr;
#ifdef _WIN32
#pragma warning(pop)
#endif
    };
    }
  }
}

#endif
