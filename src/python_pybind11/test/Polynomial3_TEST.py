# Copyright (C) 2022 Open Source Robotics Foundation
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
from ignition.math import Polynomial3d
from ignition.math import Vector4d


class TestPolynomial3(unittest.TestCase):

    def test_default_constructor(self):
        poly = Polynomial3d()
        self.assertEqual(poly.coeffs(), Vector4d.ZERO)

    def test_constructor(self):
        poly = Polynomial3d(Vector4d.ONE)
        self.assertEqual(poly.coeffs(), Vector4d.ONE)

    def test_construction_helpers(self):
        poly = Polynomial3d.CONSTANT(1.)
        self.assertEqual(poly.coeffs(), Vector4d(0., 0., 0., 1.))

    def test_evaluate(self):
        p = Polynomial3d.CONSTANT(1.0)
        self.assertEqual(p(-1.0), 1.0)
        self.assertEqual(p(0.0), 1.0)
        self.assertEqual(p(1.0), 1.0)
        self.assertEqual(p(math.inf), 1.0)
        self.assertTrue(math.isnan(p(math.nan)))

        p = Polynomial3d(Vector4d.ONE)
        self.assertEqual(p(-1.0), 0.0)
        self.assertEqual(p(0.0), 1.0)
        self.assertEqual(p(1.0), 4.0)
        self.assertEqual(p(-math.inf), -math.inf)
        self.assertTrue(math.isnan(p(math.nan)))

    # TODO(aditya) - port Interval to python.
    def test_minimum(self):
        pass

    # TODO
    def test_polynomial_stream_out(self):
        p = Polynomial3d(Vector4d.ZERO)
        self.assertEqual(str(p), "0")

        p = Polynomial3d(Vector4d.ONE)
        self.assertEqual(str(p), "x^3 + x^2 + x + 1")

        p = Polynomial3d(Vector4d(1, 0, 1, 0))
        self.assertEqual(str(p), "x^3 + x")

        p = Polynomial3d(Vector4d(0, 1, 0, -1))
        self.assertEqual(str(p), "x^2 - 1")

        p = Polynomial3d(Vector4d(-1, 2, -2, 0))
        self.assertEqual(str(p), "-x^3 + 2 x^2 - 2 x")


if __name__ == '__main__':
    unittest.main()
