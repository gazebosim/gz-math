/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
//! [complete]
#include <iostream>
#include <chrono>

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/DiffDriveOdometry.hh>

int main(int argc, char **argv)
{

//! [Create a DiffDriveOdometry]
  ignition::math::DiffDriveOdometry odom;
//! [Create an DiffDriveOdometry]

  double wheelSeparation = 2.0;
  double wheelRadius = 0.5;
  double wheelCircumference = 2 * IGN_PI * wheelRadius;

  // This is the linear distance traveled per degree of wheel rotation.
  double distPerDegree = wheelCircumference / 360.0;

  odom.SetWheelParams(wheelSeparation, wheelRadius, wheelRadius);
  auto startTime = std::chrono::steady_clock::now();
  odom.Init(startTime);

  // Sleep for a little while, then update the odometry with the new wheel
  // position.
  std::cout << "--- Rotate the both wheels by 1 degree. ---" << '\n';
  auto time1 = startTime + std::chrono::milliseconds(100);
  odom.Update(IGN_DTOR(1.0), IGN_DTOR(1.0), time1);

  // Linear velocity should be dist_traveled / time_elapsed.
  std::cout << "\tLinear velocity: " << distPerDegree / 0.1
            << " Odom linear velocity: " << odom.LinearVelocity() << std::endl;

  // Angular velocity should be zero since the "robot" is traveling in a
  // straight line.
  std::cout << "Angular velocity should be zero since the 'robot' is traveling"
            << " in a straight line:\n"
            << "\tOdom angular velocity: "
            << *odom.AngularVelocity() << std::endl;

  // Sleep again, this time rotate the right wheel by 1 degree.
  std::cout << "--- This time rotate the right wheel by 1 degree. ---"
            << std::endl;
  auto time2 = time1 + std::chrono::milliseconds(100);
  odom.Update(IGN_DTOR(2.0), IGN_DTOR(3.0), time2);

  // The heading should be the arc tangent of the linear distance traveled
  // by the right wheel (the left wheel was stationary) divided by the
  // wheel separation.
  std::cerr << "The heading should be the arc tangent of the linear distance"
            << " traveled by the right wheel (the left wheel was stationary)"
            << " divided by the wheel separation.\n"
            << "\tHeading: " << atan2(distPerDegree, wheelSeparation)
            << " Odom Heading " << *odom.Heading() << '\n';

  // The X odom reading should have increased by the sine of the heading *
  // half the wheel separation.
  double xDistTraveled =
    sin(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevXPos = distPerDegree * 2.0;
  std::cout << "\tX distance traveled " << xDistTraveled + prevXPos
             << " Odom X: " << odom.X() << std::endl;

  // The Y odom reading should have increased by the cosine of the header *
  // half the wheel separation.
  double yDistTraveled = (wheelSeparation * 0.5) -
      cos(atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5;
  double prevYPos = 0.0;
  std::cout << "\tY distance traveled " << yDistTraveled + prevYPos
             << " Odom Y: " << odom.Y() << std::endl;

  // Angular velocity should be the difference between the x and y distance
  // traveled divided by the wheel separation divided by the seconds
  // elapsed.
  std::cout << "Angular velocity should be the difference between the x and y"
            << " distance traveled divided by the wheel separation divided by"
            << " the seconds elapsed.\n"
            << "\tAngular velocity "
            << ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1
            << " Odom angular velocity: " << *odom.AngularVelocity() << std::endl;
}
//! [complete]
