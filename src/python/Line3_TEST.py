# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
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
from ignition.math import Line3d
from ignition.math import Vector3d


class TestLine3d(unittest.TestCase):

    def test_construction(self):
        lineA = Line3d(0, 0, 10, 10)
        self.assertAlmostEqual(lineA[0].x(), 0.0)
        self.assertAlmostEqual(lineA[0].y(), 0.0)
        self.assertAlmostEqual(lineA[0].z(), 0.0)
        self.assertAlmostEqual(lineA[1].x(), 10.0)
        self.assertAlmostEqual(lineA[1].y(), 10.0)
        self.assertAlmostEqual(lineA[1].z(), 0.0)
        lineB = Line3d(Vector3d(1, 2, 3), Vector3d(4, 5, 6))
        self.assertAlmostEqual(lineB[0].x(), 1.0)
        self.assertAlmostEqual(lineB[0].y(), 2.0)
        self.assertAlmostEqual(lineB[0].z(), 3.0)
        self.assertAlmostEqual(lineB[1].x(), 4.0)
        self.assertAlmostEqual(lineB[1].y(), 5.0)
        self.assertAlmostEqual(lineB[1].z(), 6.0)

        lineC = Line3d(0, 0, 5, 10, 10, 6)
        self.assertAlmostEqual(lineC[0].x(), 0.0)
        self.assertAlmostEqual(lineC[0].y(), 0.0)
        self.assertAlmostEqual(lineC[0].z(), 5.0)
        self.assertAlmostEqual(lineC[1].x(), 10.0)
        self.assertAlmostEqual(lineC[1].y(), 10.0)
        self.assertAlmostEqual(lineC[1].z(), 6.0)

        self.assertAlmostEqual(lineB[2].x(), lineB[1].x())

    def test_set(self):
        lineA = Line3d()
        lineA.set(1, 1, 2, 2)
        self.assertAlmostEqual(lineA[0].x(), 1.0)
        self.assertAlmostEqual(lineA[0].y(), 1.0)
        self.assertAlmostEqual(lineA[0].z(), 0.0)
        self.assertAlmostEqual(lineA[1].x(), 2.0)
        self.assertAlmostEqual(lineA[1].y(), 2.0)
        self.assertAlmostEqual(lineA[1].z(), 0.0)

        lineA.set(10, 11, 12, 13, 14, 15)
        self.assertAlmostEqual(lineA[0].x(), 10.0)
        self.assertAlmostEqual(lineA[0].y(), 11.0)
        self.assertAlmostEqual(lineA[0].z(), 12.0)
        self.assertAlmostEqual(lineA[1].x(), 13.0)
        self.assertAlmostEqual(lineA[1].y(), 14.0)
        self.assertAlmostEqual(lineA[1].z(), 15.0)

        lineA.set_a(Vector3d(0, -1, -2))
        self.assertAlmostEqual(lineA[0].x(), 0.0)
        self.assertAlmostEqual(lineA[0].y(), -1.0)
        self.assertAlmostEqual(lineA[0].z(), -2.0)
        self.assertAlmostEqual(lineA[1].x(), 13.0)
        self.assertAlmostEqual(lineA[1].y(), 14.0)
        self.assertAlmostEqual(lineA[1].z(), 15.0)

        lineA.set_b(Vector3d(5, 6, 7))
        self.assertAlmostEqual(lineA[0].x(), 0.0)
        self.assertAlmostEqual(lineA[0].y(), -1.0)
        self.assertAlmostEqual(lineA[0].z(), -2.0)
        self.assertAlmostEqual(lineA[1].x(), 5.0)
        self.assertAlmostEqual(lineA[1].y(), 6.0)
        self.assertAlmostEqual(lineA[1].z(), 7.0)

    def test_length(self):
        lineA = Line3d(0, 0, 0, 10, 10, 10)
        self.assertAlmostEqual(lineA.length(), math.sqrt(300), delta=1e-10)

    def test_equality(self):
        lineA = Line3d(1, 1, 1, 2, 1, 2)
        lineB = Line3d(1, 2, 3, 2, 2, 4)

        self.assertTrue(lineA != lineB)
        self.assertTrue(lineA == lineA)

        lineB.set(1, 1, 1, 2, 1.1, 2)
        self.assertFalse(lineA == lineB)

        lineB.set(1, 1, 1, 2.1, 1, 2)
        self.assertFalse(lineA == lineB)

        lineB.set(1, 1, 1.1, 2, 1, 2)
        self.assertFalse(lineA == lineB)

        lineB.set(1.1, 1, 1, 2, 1, 2)
        self.assertFalse(lineA == lineB)

    def test_serialization(self):
        line = Line3d(0, 1, 4, 2, 3, 7)
        self.assertEqual(str(line), "0 1 4 2 3 7")

    def test_copy_constructor(self):
        lineA = Line3d(0, 1, 4, 2, 3, 7)
        lineB = Line3d(lineA)

        self.assertEqual(lineA, lineB)

    def test_direction(self):
        lineA = Line3d(1, 1, 1, 0, 0, 0)
        lineB = Line3d(2, 2, 2, 0, 0, 0)
        lineC = Line3d(0, 0, 0, 1, 1, 1)
        self.assertTrue(lineA.direction() == (lineA[1] - lineA[0]).normalize())
        self.assertTrue(lineA.direction() == lineB.direction())
        self.assertFalse(lineA.direction() == lineC.direction())

        lineA.set(1, 1, 2, 1, 1, 10)
        self.assertTrue(lineA.direction() == Vector3d.UNIT_Z)

        lineA.set(1, 5, 1, 1, 1, 1)
        self.assertTrue(lineA.direction() == -Vector3d.UNIT_Y)

        lineA.set(1, 1, 1, 7, 1, 1)
        self.assertTrue(lineA.direction() == Vector3d.UNIT_X)

    def test_within(self):
        line = Line3d(0, 0, 0, 1, 1, 1)
        self.assertTrue(line.within(Vector3d(0, 0, 0)))
        self.assertTrue(line.within(Vector3d(1, 1, 1)))
        self.assertTrue(line.within(Vector3d(0.5, 0.5, 0.5)))

        self.assertFalse(line.within(Vector3d(-0.5, 0.5, 0.5)))
        self.assertFalse(line.within(Vector3d(0.5, -0.5, 0.5)))
        self.assertFalse(line.within(Vector3d(0.5, 0.5, -0.5)))

    def test_distance(self):
        line = Line3d(0, 0, 0, 0, 1, 0)
        result = Line3d()

        self.assertTrue(line.distance(Line3d(1, 0.5, 0, -1, 0.5, 0), result))
        self.assertAlmostEqual(result.length(), 0)
        self.assertEqual(result, Line3d(0, 0.5, 0, 0, 0.5, 0))

        self.assertTrue(line.distance(Line3d(1, 0, 0, -1, 0, 0), result))
        self.assertAlmostEqual(result.length(), 0)
        self.assertEqual(result, Line3d(0, 0, 0, 0, 0, 0))

        self.assertTrue(line.distance(Line3d(1, 1.1, 0, -1, 1.1, 0), result))
        self.assertAlmostEqual(result.length(), 0.1, delta=1e-4)
        self.assertEqual(result, Line3d(0, 1, 0, 0, 1.1, 0))

        self.assertTrue(line.distance(Line3d(1, 0.5, 0.4, -1, 0.5, 0.4),
                                      result))
        self.assertAlmostEqual(result.length(), 0.4, delta=1e-4)
        self.assertEqual(result, Line3d(0, 0.5, 0, 0, 0.5, 0.4))

        self.assertTrue(line.distance(Line3d(0, 0.5, 1, 1, 0.5, 0),
                                      result))
        self.assertAlmostEqual(result.length(), math.sin(math.pi / 4),
                               delta=1e-4)
        self.assertEqual(result, Line3d(0, 0.5, 0, 0.5, 0.5, 0.5))

        # Expect true when lines are parallel
        self.assertTrue(line.distance(Line3d(2, 0, 0, 2, 1, 0), result))
        self.assertEqual(result[0], line[0])
        self.assertEqual(result[1], Vector3d(2, 0, 0))

        self.assertTrue(line.distance(Line3d(2, 1, 0, 2, 0, 0), result))
        self.assertEqual(result[0], line[0])
        self.assertEqual(result[1], Vector3d(2, 0, 0))

        self.assertTrue(line.distance(Line3d(1, 1, 0, 1, 2, 0), result))
        self.assertEqual(result[0], line[1])
        self.assertEqual(result[1], Vector3d(1, 1, 0))

        self.assertTrue(line.distance(Line3d(1, 2, 0, 1, 1, 0), result))
        self.assertEqual(result[0], line[1])
        self.assertEqual(result[1], Vector3d(1, 1, 0))

        # Expect false when the passed in line is a point
        self.assertFalse(line.distance(Line3d(2, 0, 0, 2, 0, 0), result))

        # Expect false when the first line is a point.
        line.set(0, 0, 0, 0, 0, 0)
        self.assertFalse(line.distance(Line3d(2, 0, 0, 2, 1, 0), result))

    def test_interesct(self):
        line = Line3d(0, 0, 0, 0, 1, 0)
        pt = Vector3d()

        self.assertTrue(line.intersect(Line3d(1, 0.5, 0, -1, 0.5, 0)))
        self.assertTrue(line.intersect(Line3d(1, 0.5, 0, -1, 0.5, 0), pt))
        self.assertEqual(pt, Vector3d(0, 0.5, 0))

        self.assertTrue(line.intersect(Line3d(1, 0, 0, -1, 0, 0)))
        self.assertTrue(line.intersect(Line3d(1, 0, 0, -1, 0, 0), pt))
        self.assertEqual(pt, Vector3d(0, 0, 0))

        self.assertTrue(line.intersect(Line3d(1, 1, 0, -1, 1, 0)))
        self.assertTrue(line.intersect(Line3d(1, 1, 0, -1, 1, 0), pt))
        self.assertEqual(pt, Vector3d(0, 1, 0))

        self.assertTrue(line.intersect(Line3d(0, 0.5, -1, 0, 0.5, 1)))
        self.assertTrue(line.intersect(Line3d(0, 0.5, -1, 0, 0.5, 1), pt))
        self.assertEqual(pt, Vector3d(0, 0.5, 0))

        self.assertTrue(line.intersect(Line3d(-1, 0.5, -1, 1, 0.5, 1)))
        self.assertTrue(line.intersect(Line3d(-1, 0.5, -1, 1, 0.5, 1), pt))
        self.assertEqual(pt, Vector3d(0, 0.5, 0))

        self.assertFalse(line.intersect(Line3d(1, 1.1, 0, -1, 1.1, 0)))
        self.assertFalse(line.intersect(Line3d(1, -0.1, 0, -1, -0.1, 0)))

        self.assertFalse(line.intersect(Line3d(0.1, 0.1, 0, 0.6, 0.6, 0)))
        self.assertFalse(line.intersect(Line3d(-0.1, 0, 0, -0.1, 1, 0)))

        self.assertTrue(line.intersect(Line3d(0, -1, 0, 0, 0.1, 0)))
        self.assertTrue(line.intersect(Line3d(0, 1, 0, 0, 1.1, 0)))

    def test_parallel(self):
        line = Line3d(0, 0, 0, 0, 1, 0)
        self.assertTrue(line.parallel(Line3d(1, 0, 0, 1, 1, 0)))
        self.assertTrue(line.parallel(Line3d(1, 1, 0, 1, 0, 0)))
        self.assertTrue(line.parallel(Line3d(0, 0, 0, 0, 10, 0)))
        self.assertTrue(line.parallel(Line3d(-100, 100, 20, -100, 200, 20)))

        self.assertFalse(line.parallel(Line3d(1, 0, 0, 1, 1, 1)))
        self.assertFalse(line.parallel(Line3d(1, 0, 0, 2, 0, 0)))
        self.assertFalse(line.parallel(Line3d(1, 0, 1, 2, 0, 1)))

    def test_coplanar(self):
        line = Line3d(0, 0, 0, 0, 1, 0)
        self.assertTrue(line.coplanar(Line3d(1, 0, 0, 1, 1, 0)))
        self.assertTrue(line.coplanar(Line3d(0, 0, 0, 0, 10, 0)))
        self.assertTrue(line.coplanar(Line3d(-100, 100, 20, -100, 200, 20)))

        self.assertFalse(line.coplanar(Line3d(1, 0, 0, 1, 1, 1)))
        self.assertFalse(line.coplanar(Line3d(1, 0, 1, 2, 0, 0)))


if __name__ == '__main__':
    unittest.main()
