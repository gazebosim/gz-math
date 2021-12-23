# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import datetime
import math

from ignition.math import Angle, DiffDriveOdometry

odom = DiffDriveOdometry()

wheelSeparation = 2.0
wheelRadius = 0.5
wheelCircumference = 2 * 3.1416 * wheelRadius

# This is the linear distance traveled per degree of wheel rotation.
distPerDegree = wheelCircumference / 360.0

# Setup the wheel parameters, and initialize
odom.set_wheel_params(wheelSeparation, wheelRadius, wheelRadius)
startTime = datetime.datetime.now()
odom.init(datetime.timedelta())

print('--- Rotate the both wheels by 1 degree. ---')
time1 = startTime + datetime.timedelta(milliseconds=100)
odom.update(Angle(1.0 * 3.1416 / 180),
            Angle(1.0 * 3.1416 / 180),
            time1 - startTime)

print('Linear velocity: {} Odom linear velocity: {}'.
    format(distPerDegree / 0.1, odom.linear_velocity()))

print('Angular velocity should be zero since the "robot" is traveling' +
      ' in a straight line:\n' +
      '\tOdom angular velocity: {}'
      .format(odom.angular_velocity()))

# Sleep again, this time rotate the right wheel by 1 degree.
print('--- This time rotate the right wheel by 1 degree. ---');
time2 = time1 + datetime.timedelta(milliseconds=100)
odom.update(Angle(2.0 * 3.1416 / 180),
            Angle(3.0 * 3.1416 / 180),
            time2 - startTime)

print('The heading should be the arc tangent of the linear distance' +
      ' traveled by the right wheel (the left wheel was stationary)' +
      ' divided by the wheel separation.\n' +
      '\tHeading: {} Odom Heading: {}'.format(
            math.atan2(distPerDegree, wheelSeparation),
                  odom.heading()))

# The X odom reading should have increased by the sine of the heading *
# half the wheel separation.
xDistTraveled = math.sin(
    math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
prevXPos = distPerDegree * 2.0
print('\tX distance traveled: {} Odom X: {}'.format(
        xDistTraveled + prevXPos, odom.x()))

# The Y odom reading should have increased by the cosine of the header *
# half the wheel separation.
yDistTraveled = (wheelSeparation * 0.5) - math.cos(
        math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
prevYPos = 0.0
print('\tY distance traveled: {} Odom Y: {}'.format(
        yDistTraveled + prevYPos, odom.y()))

# Angular velocity should be the difference between the x and y distance
# traveled divided by the wheel separation divided by the seconds
# elapsed.
print('Angular velocity should be the difference between the x and y' +
      ' distance traveled divided by the wheel separation divided by' +
      ' the seconds elapsed.\n' +
      '\tAngular velocity: {} Odom angular velocity: {}'.format(
        ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1,
        odom.angular_velocity()))
