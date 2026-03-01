/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <cmath>
#include "gz/math/FourwidsDriveOdometry.hh"
#include "gz/math/RollingMean.hh"

#include <Eigen/Dense>
#include <iostream>

// And these calculations are based on the following references:
// https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#swerve-drive-robots:~:text=Swerve%20Drive%20Robots%EF%83%81

namespace gz
{
namespace math
{
inline namespace GZ_MATH_VERSION_NAMESPACE
{
class FourwidsDriveOdometry::Implementation
{
  /// \brief Constructor.
  /// \param[in] _windowSize Rolling window size used to compute the
  /// velocity mean.
  public: explicit Implementation(size_t _windowSize)
    : linearMean(_windowSize), lateralMean(_windowSize),
      angularMean(_windowSize)
  {
  }

  /// \brief Integrates the pose.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _lateral, double _angular);

  /// \brief Integrates the pose using second order Runge-Kuffa approximation.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear, double _lateral,
    double _angular);

  /// \brief Current timestamp.
  public: FourwidsDriveOdometry::clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: gz::math::Angle heading;

  /// \brief Current linear velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current lateral velocity in meter/second.
  public: double lateralVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: gz::math::Angle angularVel;

  /// \brief Wheel radius in meters.
  public: double wheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{1.0};

  /// \brief Wheel base in meters.
  public: double wheelBase{1.0};

  /// \brief Previous frontleft wheel position/state in radians.
  public: double frontLeftWheelOldPos{0.0};

  /// \brief Previous frontright wheel position/state in radians.
  public: double frontRightWheelOldPos{0.0};

  /// \brief Previous backleft wheel position/state in radians.
  public: double backLeftWheelOldPos{0.0};

  /// \brief Previous backright wheel position/state in radians.
  public: double backRightWheelOldPos{0.0};

  /// \brief Previous frontleft steering position/state in radians.
  public: double frontLeftSteeringOldPos{0.0};

  /// \brief Previous frontright steering position/state in radians.
  public: double frontRightSteeringOldPos{0.0};

  /// \brief Previous backleft steering position/state in radians.
  public: double backLeftSteeringOldPos{0.0};

  /// \brief Previous backright steering position/state in radians.
  public: double backRightSteeringOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: gz::math::RollingMean linearMean;

  /// \brief Rolling mean accumulators for the lateral velocity
  public: gz::math::RollingMean lateralMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: gz::math::RollingMean angularMean;

  /// \brief Initialized flag.
  public: bool initialized{false};
};
}  // namespace GZ_MATH_VERSION_NAMESPACE
}  // namespace math
}  // namespace gz

using namespace gz;
using namespace math;

//////////////////////////////////////////////////
FourwidsDriveOdometry::FourwidsDriveOdometry(size_t _windowSize)
  : dataPtr(gz::utils::MakeImpl<Implementation>(_windowSize))
{
}

void FourwidsDriveOdometry::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearMean.Clear();
  this->dataPtr->lateralMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVel = 0.0;
  this->dataPtr->lateralVel = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->frontLeftWheelOldPos = 0.0;
  this->dataPtr->frontRightWheelOldPos = 0.0;
  this->dataPtr->backLeftWheelOldPos = 0.0;
  this->dataPtr->backRightWheelOldPos = 0.0;
  this->dataPtr->frontLeftSteeringOldPos = 0.0;
  this->dataPtr->frontRightSteeringOldPos = 0.0;
  this->dataPtr->backLeftSteeringOldPos = 0.0;
  this->dataPtr->backRightSteeringOldPos = 0.0;

  this->dataPtr->lastUpdateTime = _time;
  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
bool FourwidsDriveOdometry::Initialized() const
{
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
bool FourwidsDriveOdometry::Update(const Angle &_frontLeftPos,
  const Angle &_frontRightPos, const Angle &_backLeftPos,
  const Angle &_backRightPos, const Angle &_frontLeftSteeringPos,
  const Angle &_frontRightSteeringPos, const Angle &_backLeftSteeringPos,
  const Angle &_backRightSteeringPos, const clock::time_point &_time)
{
  if (!this->dataPtr->initialized)
  {
    return false;
  }

  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;

  // Get current wheel joint positions:
  const double frontLeftWheelCurPos =
    *_frontLeftPos * this->dataPtr->wheelRadius;
  const double frontRightWheelCurPos =
    *_frontRightPos * this->dataPtr->wheelRadius;
  const double backLeftWheelCurPos =
    *_backLeftPos * this->dataPtr->wheelRadius;
  const double backRightWheelCurPos =
    *_backRightPos * this->dataPtr->wheelRadius;

  // Estimate velocity of wheels using old and current position:
  const double frontLeftWheelEstVel = frontLeftWheelCurPos -
                                 this->dataPtr->frontLeftWheelOldPos;

  const double frontRightWheelEstVel = frontRightWheelCurPos -
                                  this->dataPtr->frontRightWheelOldPos;

  const double backLeftWheelEstVel = backLeftWheelCurPos -
                                 this->dataPtr->backLeftWheelOldPos;

  const double backRightWheelEstVel = backRightWheelCurPos -
                                  this->dataPtr->backRightWheelOldPos;

  // Update old position with current
  this->dataPtr->frontLeftWheelOldPos = frontLeftWheelCurPos;
  this->dataPtr->frontRightWheelOldPos = frontRightWheelCurPos;
  this->dataPtr->backLeftWheelOldPos = backLeftWheelCurPos;
  this->dataPtr->backRightWheelOldPos = backRightWheelCurPos;

  // Handle the degenerate case where all wheel velocities align,
  // reducing the motion to pure translation.
  const double frontLeftWheelVel = frontLeftWheelEstVel / dt.count();
  const double frontRightWheelVel = frontRightWheelEstVel / dt.count();
  const double backLeftWheelVel = backLeftWheelEstVel / dt.count();
  const double backRightWheelVel = backRightWheelEstVel / dt.count();

  Eigen::Vector2d frontLeftWheelVelVec(
    frontLeftWheelVel * std::cos(*_frontLeftSteeringPos),
    frontLeftWheelVel * std::sin(*_frontLeftSteeringPos));
  Eigen::Vector2d frontRightWheelVelVec(
    frontRightWheelVel * std::cos(*_frontRightSteeringPos),
    frontRightWheelVel * std::sin(*_frontRightSteeringPos));
  Eigen::Vector2d backLeftWheelVelVec(
    backLeftWheelVel * std::cos(*_backLeftSteeringPos),
    backLeftWheelVel * std::sin(*_backLeftSteeringPos));
  Eigen::Vector2d backRightWheelVelVec(
    backRightWheelVel * std::cos(*_backRightSteeringPos),
    backRightWheelVel * std::sin(*_backRightSteeringPos));

  Eigen::Vector2d meanVec = (frontLeftWheelVelVec + frontRightWheelVelVec +
    backLeftWheelVelVec + backRightWheelVelVec) / 4.0;
  double maxDev = std::max({(frontLeftWheelVelVec - meanVec).norm(),
                             (frontRightWheelVelVec - meanVec).norm(),
                             (backLeftWheelVelVec - meanVec).norm(),
                             (backRightWheelVelVec - meanVec).norm()});
  double meanNorm = meanVec.norm();

  bool vectors_identical = false;
  if (maxDev < 3e-3)
  {
    vectors_identical = true;
  }
  else if (meanNorm > 1e-6 && (maxDev / meanNorm) < 1e-2)
  {
    vectors_identical = true;
  }

  Eigen::MatrixXd A(4, 3);
  Eigen::VectorXd s(4);

  const double halfWheelBase = 0.5 * this->dataPtr->wheelBase;
  const double halfWheelSeparation = 0.5 * this->dataPtr->wheelSeparation;
  A(0, 0) = std::cos(*_frontLeftSteeringPos);
  A(0, 1) = std::sin(*_frontLeftSteeringPos);
  A(0, 2) = std::sin(*_frontLeftSteeringPos) * halfWheelBase -
            std::cos(*_frontLeftSteeringPos) * halfWheelSeparation;
  A(1, 0) = std::cos(*_frontRightSteeringPos);
  A(1, 1) = std::sin(*_frontRightSteeringPos);
  A(1, 2) = std::sin(*_frontRightSteeringPos) * halfWheelBase -
            std::cos(*_frontRightSteeringPos) * -1.0 * halfWheelSeparation;
  A(2, 0) = std::cos(*_backLeftSteeringPos);
  A(2, 1) = std::sin(*_backLeftSteeringPos);
  A(2, 2) = std::sin(*_backLeftSteeringPos) * -1.0 * halfWheelBase -
            std::cos(*_backLeftSteeringPos) * halfWheelSeparation;
  A(3, 0) = std::cos(*_backRightSteeringPos);
  A(3, 1) = std::sin(*_backRightSteeringPos);
  A(3, 2) = std::sin(*_backRightSteeringPos) * -1.0 * halfWheelBase -
            std::cos(*_backRightSteeringPos) * -1.0 * halfWheelSeparation;

  s(0) = frontLeftWheelEstVel / dt.count();
  s(1) = frontRightWheelEstVel / dt.count();
  s(2) = backLeftWheelEstVel / dt.count();
  s(3) = backRightWheelEstVel / dt.count();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    A,
    Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singularValues = svd.singularValues();
  Eigen::Vector3d v;

  int rank = (singularValues.array() > 1e-6).count();

  if (vectors_identical)
  {
    v = Eigen::Vector3d(meanVec(0), meanVec(1), 0.0);
  }
  else if (rank < 3)
  {
    // If the system is rank-deficient, then first assume
    // zero angular velocity and solve for (vx, vy);
    Eigen::MatrixXd ADegrade = A.leftCols(2);
    Eigen::JacobiSVD<Eigen::MatrixXd> svdDegrade(
      ADegrade,
      Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValuesDegrade = svdDegrade.singularValues();
    int rankDegrade = (singularValuesDegrade.array() > 1e-6).count();

    if (rankDegrade >= 2)
    {
      Eigen::Vector2d vDegrade = svdDegrade.solve(s);
      v << vDegrade(0), vDegrade(1), 0.0;
    }
    else
    {
      // If still unsolvable, then apply regularized least-squares.
      double lambda = 1e-3;
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
      v = (A.transpose() * A + lambda * I).ldlt().solve(A.transpose() * s);
    }
  }
  else
  {
    v = svd.solve(s);
  }

  // Integrate the pose
  this->dataPtr->IntegrateExact(
    v(0) * dt.count(),
    v(1) * dt.count(),
    v(2) * dt.count());

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearMean.Push(v(0));
  this->dataPtr->lateralMean.Push(v(1));
  this->dataPtr->angularMean.Push(v(2));

  this->dataPtr->linearVel = this->dataPtr->linearMean.Mean();
  this->dataPtr->lateralVel = this->dataPtr->lateralMean.Mean();
  this->dataPtr->angularVel = this->dataPtr->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::SetWheelParams(double _wheelSeparation,
  double _wheelBase, double _wheelRadius)
{
  this->dataPtr->wheelSeparation = _wheelSeparation;
  this->dataPtr->wheelBase = _wheelBase;
  this->dataPtr->wheelRadius = _wheelRadius;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearMean.SetWindowSize(_size);
  this->dataPtr->lateralMean.SetWindowSize(_size);
  this->dataPtr->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const Angle &FourwidsDriveOdometry::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::LinearVelocity() const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::LateralVelocity() const
{
  return this->dataPtr->lateralVel;
}

//////////////////////////////////////////////////
const Angle &FourwidsDriveOdometry::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelSeparation() const
{
  return this->dataPtr->wheelSeparation;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelBase() const
{
  return this->dataPtr->wheelBase;
}

//////////////////////////////////////////////////
double FourwidsDriveOdometry::WheelRadius() const
{
  return this->dataPtr->wheelRadius;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::Implementation::IntegrateRungeKutta2(
    double _linear, double _lateral, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += (_linear * std::cos(direction)) - (_lateral * std::sin(direction));
  this->y += _linear * std::sin(direction) + (_lateral * std::cos(direction));
  this->heading += _angular;
}

//////////////////////////////////////////////////
void FourwidsDriveOdometry::Implementation::IntegrateExact(double _linear,
  double _lateral, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _lateral, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    const double ratio2 = _lateral / _angular;
    this->heading += _angular;
    this->x += (ratio * (std::sin(*this->heading) - std::sin(headingOld)))
      - (-ratio2 * (std::cos(*this->heading) - std::cos(headingOld)));
    this->y += (-ratio * (std::cos(*this->heading) - std::cos(headingOld)))
      + (ratio2 * (std::sin(*this->heading) - std::sin(headingOld)));
  }
}
