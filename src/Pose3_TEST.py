# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import math
from ignition.math import Pose3d
from ignition.math import Quaterniond
from ignition.math import Vector3d


class TestPose3(unittest.TestCase):
    def test_construction(self):
        pose = Pose3d(1, 0, 0, 0, 0, 0)

        # Copy
        pose2 = Pose3d(pose)
        self.assertAlmostEqual(pose2, pose)

        # Inequality
        pose3 = Pose3d()
        self.assertNotEqual(pose3, pose)

    def test_pose(self):
        A = Pose3d(Vector3d(1, 0, 0),
                   Quaterniond(0, 0, math.pi/4.0))
        B = Pose3d(Vector3d(1, 0, 0),
                   Quaterniond(0, 0, math.pi/2.0))

        # test hypothesis that if
        # A is the transform from O to P specified in frame O
        # B is the transform from P to Q specified in frame P
        # then, B + A is the transform from O to Q specified in frame O
        self.assertAlmostEqual((B + A).Pos().X(), 1.0 + 1.0/math.sqrt(2))
        self.assertAlmostEqual((B + A).Pos().Y(), 1.0/math.sqrt(2))
        self.assertAlmostEqual((B + A).Pos().Z(), 0.0)
        self.assertAlmostEqual((B + A).Rot().Euler().X(), 0.0)
        self.assertAlmostEqual((B + A).Rot().Euler().Y(), 0.0)
        self.assertAlmostEqual((B + A).Rot().Euler().Z(), 3.0*math.pi/4.0)

        # If:
        # A is the transform from O to P in frame O
        # B is the transform from O to Q in frame O
        # then -A is transform from P to O specified in frame P
        self.assertAlmostEqual((Pose3d() - A).Pos().X(), -1.0/math.sqrt(2))
        self.assertAlmostEqual((Pose3d() - A).Pos().Y(), 1.0/math.sqrt(2))
        self.assertAlmostEqual((Pose3d() - A).Pos().Z(), 0.0)
        self.assertAlmostEqual((Pose3d() - A).Rot().Euler().X(),  0.0)
        self.assertAlmostEqual((Pose3d() - A).Rot().Euler().Y(),  0.0)
        self.assertAlmostEqual((Pose3d() - A).Rot().Euler().Z(), -math.pi/4)

        # test negation operator
        self.assertAlmostEqual((-A).Pos().X(), -1.0/math.sqrt(2))
        self.assertAlmostEqual((-A).Pos().Y(), 1.0/math.sqrt(2))
        self.assertAlmostEqual((-A).Pos().Z(), 0.0)
        self.assertAlmostEqual((-A).Rot().Euler().X(), 0.0)
        self.assertAlmostEqual((-A).Rot().Euler().Y(), 0.0)
        self.assertAlmostEqual((-A).Rot().Euler().Z(), -math.pi/4.0)

        # If:
        # A is the transform from O to P in frame O
        # B is the transform from O to Q in frame O
        # B - A is the transform from P to Q in frame P
        B = Pose3d(Vector3d(1, 1, 0),
                   Quaterniond(0, 0, math.pi/2.0))
        self.assertAlmostEqual((B - A).Pos().X(), 1.0/math.sqrt(2))
        self.assertAlmostEqual((B - A).Pos().Y(), 1.0/math.sqrt(2))
        self.assertAlmostEqual((B - A).Pos().Z(), 0.0)
        self.assertAlmostEqual((B - A).Rot().Euler().X(), 0.0)
        self.assertAlmostEqual((B - A).Rot().Euler().Y(), 0.0)
        self.assertAlmostEqual((B - A).Rot().Euler().Z(), math.pi/4.0)

        pose = Pose3d()
        self.assertTrue(pose.Pos() == Vector3d(0, 0, 0))
        self.assertTrue(pose.Rot() == Quaterniond(0, 0, 0))

        pose = Pose3d(Vector3d(1, 2, 3), Quaterniond(.1, .2, .3))
        self.assertTrue(pose.Pos() == Vector3d(1, 2, 3))
        self.assertTrue(pose.Rot() == Quaterniond(.1, .2, .3))

        pose1 = Pose3d(pose)
        self.assertTrue(pose1 == pose)

        pose.Set(Vector3d(2, 3, 4), Quaterniond(.3, .4, .5))
        self.assertTrue(pose.Pos() == Vector3d(2, 3, 4))
        self.assertTrue(pose.Rot() == Quaterniond(.3, .4, .5))
        self.assertTrue(pose.IsFinite())

        pose1 = pose.Inverse()
        self.assertTrue(pose1.Pos() == Vector3d(-1.38368, -3.05541, -4.21306))
        self.assertTrue(pose1.Rot() == Quaterniond(0.946281, -0.0933066,
                                                   -0.226566, -0.210984))

        pose = Pose3d(1, 2, 3, .1, .2, .3) + Pose3d(4, 5, 6, .4, .5, .6)
        self.assertTrue(pose == Pose3d(5.74534, 7.01053, 8.62899,
                                       0.675732, 0.535753, 1.01174))

        pose += pose
        self.assertTrue(pose == Pose3d(11.314, 16.0487, 15.2559,
                                       1.49463, 0.184295, 2.13932))

        pose -= Pose3d(pose)
        self.assertTrue(pose == Pose3d(0, 0, 0, 0, 0, 0))

        pose.Pos().Set(5, 6, 7)
        pose.Rot().Euler(Vector3d(.4, .6, 0))

        self.assertTrue(pose.CoordPositionAdd(Vector3d(1, 2, 3)) ==
                        Vector3d(7.82531, 6.67387, 9.35871))

        self.assertTrue(pose.CoordPositionAdd(pose1) ==
                        Vector3d(2.58141, 2.4262, 3.8013))
        self.assertTrue(pose.CoordRotationAdd(Quaterniond(0.1, 0, 0.2)) ==
                        Quaterniond(0.520975, 0.596586, 0.268194))
        self.assertTrue(pose.CoordPoseSolve(pose1) ==
                        Pose3d(-0.130957, -11.552, -10.2329,
                        -0.462955, -1.15624, -0.00158047))

        self.assertTrue(pose.RotatePositionAboutOrigin(
                        Quaterniond(0.1, 0, 0.2)) ==
                        Pose3d(6.09235, 5.56147, 6.47714, 0.4, 0.6, 0))

        pose.Reset()
        self.assertTrue(pose.Pos() == Vector3d(0, 0, 0))
        self.assertTrue(pose.Rot() == Quaterniond(0, 0, 0))

    def test_pose_atributes(self):
        pose = Pose3d(0, 1, 2, 1, 0, 0)

        self.assertTrue(pose.Pos() == Vector3d(0, 1, 2))
        self.assertTrue(pose.Rot() == Quaterniond(1, 0, 0))

    def test_stream_out(self):
        p = Pose3d(0.1, 1.2, 2.3, 0.0, 0.1, 1.0)
        self.assertAlmostEqual(str(p), "0.1 1.2 2.3 0 0.1 1")

    def test_mutable_pose(self):
        pose = Pose3d(0, 1, 2, 0, 0, 0)

        self.assertTrue(pose.Pos() == Vector3d(0, 1, 2))
        self.assertTrue(pose.Rot() == Quaterniond(0, 0, 0))

        pose = Pose3d(Vector3d(10, 20, 30), Quaterniond(1, 2, 1))

        self.assertTrue(pose.Pos() == Vector3d(10, 20, 30))
        self.assertTrue(pose.Rot() == Quaterniond(1, 2, 1))

    def test_pose_elements(self):
        pose = Pose3d(0, 1, 2, 1, 1, 2)
        self.assertAlmostEqual(pose.X(), 0)
        self.assertAlmostEqual(pose.Y(), 1)
        self.assertAlmostEqual(pose.Z(), 2)
        self.assertAlmostEqual(pose.Roll(), 1)
        self.assertAlmostEqual(pose.Pitch(), 1)
        self.assertAlmostEqual(pose.Yaw(), 2)

    def test_set_pose_element(self):
        pose = Pose3d(1, 2, 3, 1.57, 1, 2)
        self.assertAlmostEqual(pose.X(), 1)
        self.assertAlmostEqual(pose.Y(), 2)
        self.assertAlmostEqual(pose.Z(), 3)

        pose.SetX(10)
        pose.SetY(12)
        pose.SetZ(13)

        self.assertAlmostEqual(pose.X(), 10)
        self.assertAlmostEqual(pose.Y(), 12)
        self.assertAlmostEqual(pose.Z(), 13)


if __name__ == '__main__':
    unittest.main()