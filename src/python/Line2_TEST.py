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
from ignition.math import Line2d
from ignition.math import Vector2d


class TestLine2d(unittest.TestCase):

    def test_construction(self):
        lineA = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(lineA[0].x(), 0.0)
        self.assertAlmostEqual(lineA[0].y(), 0.0)
        self.assertAlmostEqual(lineA[1].x(), 10.0)
        self.assertAlmostEqual(lineA[1].y(), 10.0)

        lineB = Line2d(Vector2d(1, 2), Vector2d(3, 4))
        self.assertAlmostEqual(lineB[0].x(), 1.0)
        self.assertAlmostEqual(lineB[0].y(), 2.0)
        self.assertAlmostEqual(lineB[1].x(), 3.0)
        self.assertAlmostEqual(lineB[1].y(), 4.0)

        self.assertAlmostEqual(lineB[2].x(), lineB[1].x())

    def test_length(self):
        lineA = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(lineA.length(), math.sqrt(200), delta=1e-10)

    def test_slope(self):
        line = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(line.slope(), 1.0, delta=1e-10)

        line = Line2d(0, 0, 0, 10)
        self.assertTrue(math.isnan(line.slope()))

        line = Line2d(-10, 0, 100, 0)
        self.assertAlmostEqual(line.slope(), 0.0)

    def test_parallel_line(self):
        # Line is always parallel with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.parallel(line, 1e-10))

        # Degenerate line segment
        # Still expect Line is parallel with itself
        line = Line2d(0, 0, 0, 0)
        self.assertTrue(line.parallel(line, 1e-10))

        lineA = Line2d(0, 0, 10, 0)
        lineB = Line2d(0, 0, 10, 0)
        self.assertTrue(lineA.parallel(lineB, 1e-10))

        lineB.set(0, 0, 0, 10)
        self.assertFalse(lineA.parallel(lineB))

        lineB.set(0, 10, 10, 10)
        self.assertTrue(lineA.parallel(lineB))

        lineB.set(0, 10, 10, 10.00001)
        self.assertFalse(lineA.parallel(lineB, 1e-10))
        self.assertFalse(lineA.parallel(lineB))
        self.assertTrue(lineA.parallel(lineB, 1e-3))

    def test_collinear_line(self):
        # Line is always collinear with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.collinear(line, 1e-10))

        lineA = Line2d(0, 0, 10, 0)
        lineB = Line2d(0, 0, 10, 0)
        self.assertTrue(lineA.collinear(lineB, 1e-10))

        lineB.set(0, 10, 10, 10)
        self.assertFalse(lineA.collinear(lineB))

        lineB.set(9, 0, 10, 0.00001)
        self.assertFalse(lineA.collinear(lineB, 1e-10))
        self.assertFalse(lineA.collinear(lineB))
        self.assertTrue(lineA.collinear(lineB, 1e-3))

    def test_collinear_point(self):
        lineA = Line2d(0, 0, 10, 0)
        pt = Vector2d(0, 0)
        self.assertTrue(lineA.collinear(pt))

        ptLine = Line2d(pt, pt)
        self.assertTrue(lineA.collinear(ptLine))

        pt.set(1000, 0)
        self.assertTrue(lineA.collinear(pt, 1e-10))

        ptLine = Line2d(pt, pt)
        self.assertTrue(lineA.parallel(ptLine))
        self.assertFalse(lineA.intersect(ptLine))
        self.assertFalse(lineA.collinear(ptLine, 1e-10))

        pt.set(10, 0)
        ptLine.set(pt, pt)
        self.assertTrue(lineA.collinear(ptLine, 1e-10))

        pt.set(0, 0.00001)
        self.assertFalse(lineA.collinear(pt))
        self.assertTrue(lineA.collinear(pt, 1e-3))

        ptLine = Line2d(pt, pt)
        self.assertFalse(lineA.collinear(ptLine))
        self.assertTrue(lineA.parallel(ptLine))
        self.assertFalse(lineA.intersect(ptLine))
        self.assertTrue(lineA.intersect(ptLine, 1e-2))
        self.assertTrue(lineA.collinear(ptLine, 1e-3))

        pt.set(0, -0.00001)
        self.assertFalse(lineA.collinear(pt))
        self.assertTrue(lineA.collinear(pt, 1e-3))

        ptLine = Line2d(pt, pt)
        self.assertFalse(lineA.collinear(ptLine))
        self.assertTrue(lineA.collinear(ptLine, 1e-4))

    def test_intersect(self):
        pt = Vector2d()

        # parallel horizontal lines
        lineA = Line2d(1, 1, 2, 1)
        lineB = Line2d(1, 2, 2, 2)
        self.assertFalse(lineA.intersect(lineB, pt))

        # parallel vertical lines
        lineA.set(1, 1, 1, 10)
        lineB.set(2, 1, 2, 10)
        self.assertFalse(lineA.intersect(lineB, pt))

        # Two lines that form an inverted T with a gap
        lineA.set(1, 1, 1, 10)
        lineB.set(0, 0, 2, 0)
        self.assertFalse(lineA.intersect(lineB, pt))

        # Two lines that form a T with a gap
        lineA.set(1, 1, 1, 10)
        lineB.set(0, 10.1, 2, 10.1)
        self.assertFalse(lineA.intersect(lineB, pt))

        # Two lines that form an inverted T with a gap
        lineA.set(0, -10, 0, 10)
        lineB.set(1, 0, 10, 0)
        self.assertFalse(lineA.intersect(lineB, pt))

        # Two lines that form a T with a gap
        lineA.set(0, -10, 0, 10)
        lineB.set(-1, 0, -10, 0)
        self.assertFalse(lineA.intersect(lineB, pt))

        # Two collinear lines, one starts where the other stopped
        lineA.set(1, 1, 1, 10)
        lineB.set(1, 10, 1, 11)
        self.assertTrue(lineA.intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(1, 10))

        # Two collinear lines, one overlaps the other
        lineA.set(0, 0, 0, 10)
        lineB.set(0, 9, 0, 11)
        self.assertTrue(lineA.intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(0, 9))

        # Two collinear lines, one overlaps the other
        lineA.set(0, 0, 0, 10)
        lineB.set(0, -10, 0, 1)
        self.assertTrue(lineA.intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(0, 1))

        # Two intersecting lines
        lineA.set(0, 0, 10, 10)
        lineB.set(0, 10, 10, 0)
        self.assertTrue(lineA.intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(5, 5))

    def test_equality(self):
        lineA = Line2d(1, 1, 2, 1)
        lineB = Line2d(1, 2, 2, 2)

        self.assertTrue(lineA != lineB)
        self.assertTrue(lineA == lineA)

        lineB.set(1, 1, 2, 1.1)
        self.assertFalse(lineA == lineB)

        lineB.set(1, 1, 2.1, 1)
        self.assertFalse(lineA == lineB)

        lineB.set(1, 1.1, 2, 1)
        self.assertFalse(lineA == lineB)

        lineB.set(1.1, 1, 2, 1)
        self.assertFalse(lineA == lineB)

    def test_serialization(self):
        line = Line2d(0, 1, 2, 3)
        self.assertEqual(str(line), "0 1 2 3")


if __name__ == '__main__':
    unittest.main()
