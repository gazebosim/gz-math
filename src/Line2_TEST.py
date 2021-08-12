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
        self.assertAlmostEqual(lineA[0].X(), 0.0)
        self.assertAlmostEqual(lineA[0].Y(), 0.0)
        self.assertAlmostEqual(lineA[1].X(), 10.0)
        self.assertAlmostEqual(lineA[1].Y(), 10.0)

        lineB = Line2d(Vector2d(1, 2), Vector2d(3, 4))
        self.assertAlmostEqual(lineB[0].X(), 1.0)
        self.assertAlmostEqual(lineB[0].Y(), 2.0)
        self.assertAlmostEqual(lineB[1].X(), 3.0)
        self.assertAlmostEqual(lineB[1].Y(), 4.0)

        self.assertAlmostEqual(lineB[2].X(), lineB[1].X())

    def test_length(self):
        lineA = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(lineA.Length(), math.sqrt(200), delta=1e-10)

    def test_slope(self):
        line = Line2d(0, 0, 10, 10)
        self.assertAlmostEqual(line.Slope(), 1.0, delta=1e-10)

        line = Line2d(0, 0, 0, 10)
        self.assertTrue(math.isnan(line.Slope()))

        line = Line2d(-10, 0, 100, 0)
        self.assertAlmostEqual(line.Slope(), 0.0)

    def test_parallel_line(self):
        # Line is always parallel with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.Parallel(line, 1e-10))

        # Degenerate line segment
        # Still expect Line is parallel with itself
        line = Line2d(0, 0, 0, 0)
        self.assertTrue(line.Parallel(line, 1e-10))

        lineA = Line2d(0, 0, 10, 0)
        lineB = Line2d(0, 0, 10, 0)
        self.assertTrue(lineA.Parallel(lineB, 1e-10))

        lineB.Set(0, 0, 0, 10)
        self.assertFalse(lineA.Parallel(lineB))

        lineB.Set(0, 10, 10, 10)
        self.assertTrue(lineA.Parallel(lineB))

        lineB.Set(0, 10, 10, 10.00001)
        self.assertFalse(lineA.Parallel(lineB, 1e-10))
        self.assertFalse(lineA.Parallel(lineB))
        self.assertTrue(lineA.Parallel(lineB, 1e-3))

    def test_collinear_line(self):
        # Line is always collinear with itself
        line = Line2d(0, 0, 10, 0)
        self.assertTrue(line.Collinear(line, 1e-10))

        lineA = Line2d(0, 0, 10, 0)
        lineB = Line2d(0, 0, 10, 0)
        self.assertTrue(lineA.Collinear(lineB, 1e-10))

        lineB.Set(0, 10, 10, 10)
        self.assertFalse(lineA.Collinear(lineB))

        lineB.Set(9, 0, 10, 0.00001)
        self.assertFalse(lineA.Collinear(lineB, 1e-10))
        self.assertFalse(lineA.Collinear(lineB))
        self.assertTrue(lineA.Collinear(lineB, 1e-3))

    def test_collinear_point(self):
        lineA = Line2d(0, 0, 10, 0)
        pt = Vector2d(0, 0)
        self.assertTrue(lineA.Collinear(pt))

        ptLine = Line2d(pt, pt)
        self.assertTrue(lineA.Collinear(ptLine))

        pt.Set(1000, 0)
        self.assertTrue(lineA.Collinear(pt, 1e-10))

        ptLine = Line2d(pt, pt)
        self.assertTrue(lineA.Parallel(ptLine))
        self.assertFalse(lineA.Intersect(ptLine))
        self.assertFalse(lineA.Collinear(ptLine, 1e-10))

        pt.Set(10, 0)
        ptLine.Set(pt, pt)
        self.assertTrue(lineA.Collinear(ptLine, 1e-10))

        pt.Set(0, 0.00001)
        self.assertFalse(lineA.Collinear(pt))
        self.assertTrue(lineA.Collinear(pt, 1e-3))

        ptLine = Line2d(pt, pt)
        self.assertFalse(lineA.Collinear(ptLine))
        self.assertTrue(lineA.Parallel(ptLine))
        self.assertFalse(lineA.Intersect(ptLine))
        self.assertTrue(lineA.Intersect(ptLine, 1e-2))
        self.assertTrue(lineA.Collinear(ptLine, 1e-3))

        pt.Set(0, -0.00001)
        self.assertFalse(lineA.Collinear(pt))
        self.assertTrue(lineA.Collinear(pt, 1e-3))

        ptLine = Line2d(pt, pt)
        self.assertFalse(lineA.Collinear(ptLine))
        self.assertTrue(lineA.Collinear(ptLine, 1e-4))

    def test_intersect(self):
        pt = Vector2d()

        # Parallel horizontal lines
        lineA = Line2d(1, 1, 2, 1)
        lineB = Line2d(1, 2, 2, 2)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Parallel vertical lines
        lineA.Set(1, 1, 1, 10)
        lineB.Set(2, 1, 2, 10)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Two lines that form an inverted T with a gap
        lineA.Set(1, 1, 1, 10)
        lineB.Set(0, 0, 2, 0)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Two lines that form a T with a gap
        lineA.Set(1, 1, 1, 10)
        lineB.Set(0, 10.1, 2, 10.1)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Two lines that form an inverted T with a gap
        lineA.Set(0, -10, 0, 10)
        lineB.Set(1, 0, 10, 0)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Two lines that form a T with a gap
        lineA.Set(0, -10, 0, 10)
        lineB.Set(-1, 0, -10, 0)
        self.assertFalse(lineA.Intersect(lineB, pt))

        # Two collinear lines, one starts where the other stopped
        lineA.Set(1, 1, 1, 10)
        lineB.Set(1, 10, 1, 11)
        self.assertTrue(lineA.Intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(1, 10))

        # Two collinear lines, one overlaps the other
        lineA.Set(0, 0, 0, 10)
        lineB.Set(0, 9, 0, 11)
        self.assertTrue(lineA.Intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(0, 9))

        # Two collinear lines, one overlaps the other
        lineA.Set(0, 0, 0, 10)
        lineB.Set(0, -10, 0, 1)
        self.assertTrue(lineA.Intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(0, 1))

        # Two intersecting lines
        lineA.Set(0, 0, 10, 10)
        lineB.Set(0, 10, 10, 0)
        self.assertTrue(lineA.Intersect(lineB, pt))
        self.assertEqual(pt, Vector2d(5, 5))

    def test_equality(self):
        lineA = Line2d(1, 1, 2, 1)
        lineB = Line2d(1, 2, 2, 2)

        self.assertTrue(lineA != lineB)
        self.assertTrue(lineA == lineA)

        lineB.Set(1, 1, 2, 1.1)
        self.assertFalse(lineA == lineB)

        lineB.Set(1, 1, 2.1, 1)
        self.assertFalse(lineA == lineB)

        lineB.Set(1, 1.1, 2, 1)
        self.assertFalse(lineA == lineB)

        lineB.Set(1.1, 1, 2, 1)
        self.assertFalse(lineA == lineB)

    def test_serialization(self):
        line = Line2d(0, 1, 2, 3)
        self.assertEqual(str(line), "0 1 2 3")


if __name__ == '__main__':
    unittest.main()
