
import datetime
import math
import time
import unittest

from gz.math import SwerveDriveOdometry, Angle

class TestSwerveDriveOdometry(unittest.TestCase):

    def test_basic_init(self):
        odom = SwerveDriveOdometry()
        self.assertAlmostEqual(0.0, odom.heading().radian())
        self.assertAlmostEqual(0.0, odom.x())
        self.assertAlmostEqual(0.0, odom.y())
        self.assertAlmostEqual(0.0, odom.linear_velocity())
        self.assertAlmostEqual(0.0, odom.lateral_velocity())
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian())
        self.assertFalse(odom.initialized())

        wheelSeparation = 0.5
        wheelBase = 0.5
        wheelRadius = 0.12

        # Setup the wheel parameters, and initialize
        odom.set_wheel_params(wheelSeparation, wheelBase, wheelRadius)
        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)

        # Sleep for a little while, then update the odometry with the new wheel
        # position.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(1.0, 1.0, 1.0, 1.0,
                    Angle(math.radians(0.0)), Angle(math.radians(0.0)),
                    Angle(math.radians(0.0)), Angle(math.radians(0.0)),
                    time1)

        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)
        self.assertAlmostEqual(0.0, odom.heading().radian())
        self.assertAlmostEqual(0.0, odom.x())
        self.assertAlmostEqual(0.0, odom.y())
        self.assertAlmostEqual(0.0, odom.linear_velocity())
        self.assertAlmostEqual(0.0, odom.lateral_velocity())
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian())
  
    def test_straight_forward(self):
        odom = SwerveDriveOdometry()

        wheelSeparation = 0.5
        wheelBase = 0.5
        wheelRadius = 0.12

        # Setup the wheel parameters, and initialize
        odom.set_wheel_params(wheelSeparation, wheelBase, wheelRadius)
        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)

        # Sleep for a little while, then update the odometry with the new wheel
        # position.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(1.0, 1.0, 1.0, 1.0,
                    Angle(math.radians(0.0)), Angle(math.radians(0.0)),
                    Angle(math.radians(0.0)), Angle(math.radians(0.0)),
                    time1)
        expectedDistTraveled = 1.0 * wheelRadius * 0.1
        self.assertAlmostEqual(expectedDistTraveled, odom.x(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.y(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.heading().radian(), delta=1e-3)
        # Linear velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(expectedDistTraveled / 0.1, odom.linear_velocity(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.lateral_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)

    def test_straight_side(self):
        odom = SwerveDriveOdometry()

        wheelSeparation = 0.5
        wheelBase = 0.5
        wheelRadius = 0.12

        # Setup the wheel parameters, and initialize
        odom.set_wheel_params(wheelSeparation, wheelBase, wheelRadius)
        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)

        # Sleep for a little while, then update the odometry with the new wheel
        # position.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(1.0, 1.0, 1.0, 1.0,
                    Angle(math.radians(90.0)), Angle(math.radians(90.0)),
                    Angle(math.radians(90.0)), Angle(math.radians(90.0)),
                    time1)
        expectedDistTraveled = 1.0 * wheelRadius * 0.1
        self.assertAlmostEqual(0.0, odom.x(), delta=1e-3)
        self.assertAlmostEqual(expectedDistTraveled, odom.y(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.heading().radian(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.linear_velocity(), delta=1e-3)
        # Lateral velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(expectedDistTraveled / 0.1,
                               odom.lateral_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)

    def test_straight_diagonal(self):
        odom = SwerveDriveOdometry()
        
        wheelSeparation = 0.5
        wheelBase = 0.5
        wheelRadius = 0.12

        # Setup the wheel parameters, and initialize
        odom.set_wheel_params(wheelSeparation, wheelBase, wheelRadius)
        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)

        # Sleep for a little while, then update the odometry with the new wheel
        # position.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(1.0, 1.0, 1.0, 1.0,
                    Angle(math.radians(45.0)), Angle(math.radians(45.0)),
                    Angle(math.radians(45.0)), Angle(math.radians(45.0)),
                    time1)
        expectedDistTraveled = 1.0 * wheelRadius * 0.1
        self.assertAlmostEqual(expectedDistTraveled / math.sqrt(2),
                               odom.x(), delta=1e-3)
        self.assertAlmostEqual(expectedDistTraveled / math.sqrt(2),
                               odom.y(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.heading().radian(), delta=1e-3)
        self.assertAlmostEqual(expectedDistTraveled / math.sqrt(2) / 0.1,
                               odom.linear_velocity(), delta=1e-3)
        self.assertAlmostEqual(expectedDistTraveled / math.sqrt(2) / 0.1,
                               odom.lateral_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)
  
    def test_rotate_in_place(self):
        odom = SwerveDriveOdometry()

        wheelSeparation = 0.5
        wheelBase = 0.5
        wheelRadius = 0.12

        # The distance between the wheel axle and the center of the vehicle
        wheelToCenter = math.sqrt((wheelSeparation * wheelSeparation) / 4
                                  + (wheelBase * wheelBase) / 4)

        # Setup the wheel parameters, and initialize
        odom.set_wheel_params(wheelSeparation, wheelBase, wheelRadius)
        startTime = datetime.timedelta(time.monotonic())
        odom.init(startTime)

        # Sleep for a little while, then update the odometry with the new wheel
        # position.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(1.0, -1.0, 1.0, -1.0,
                    Angle(math.radians(-45.0)), Angle(math.radians(45.0)),
                    Angle(math.radians(45.0)), Angle(math.radians(-45.0)),
                    time1)
        expectedAngularTraveled = -1.0 * wheelRadius / wheelToCenter * 0.1;
        self.assertAlmostEqual(0.0, odom.x(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.y(), delta=1e-3)
        self.assertAlmostEqual(expectedAngularTraveled,
                               odom.heading().radian(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.linear_velocity(), delta=1e-3)
        self.assertAlmostEqual(0.0, odom.lateral_velocity(), delta=1e-3)
        # Angular velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(expectedAngularTraveled / 0.1,
                               odom.angular_velocity().radian(), delta=1e-3)

if __name__ == '__main__':
    unittest.main()
