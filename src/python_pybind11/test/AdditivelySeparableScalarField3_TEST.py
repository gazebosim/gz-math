# Copyright (C) 2026 Open Source Robotics Foundation
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

import copy
import math
import unittest

from gz.math import AdditivelySeparableScalarField3d
from gz.math import AdditivelySeparableScalarField3f
from gz.math import Intervald, Polynomial3d, Polynomial3f, Region3d
from gz.math import Vector3d, Vector3f, Vector4d, Vector4f


class TestAdditivelySeparableScalarField3(unittest.TestCase):

    def setUp(self):
        polynomial = Polynomial3d(Vector4d(0., 1., 1., 1.))
        self.field = AdditivelySeparableScalarField3d(
            1. / 3., polynomial, polynomial, polynomial)

    def test_evaluate(self):
        self.assertEqual(self.field.evaluate(Vector3d.ZERO), 1.)
        self.assertEqual(self.field(Vector3d.ONE), 3.)

        polynomial = Polynomial3f(Vector4f(0., 1., 1., 1.))
        field = AdditivelySeparableScalarField3f(
            1. / 3., polynomial, polynomial, polynomial)
        self.assertEqual(field(Vector3f.ONE), 3.)

    def test_minimum(self):
        self.assertEqual(self.field.minimum(), 0.75)

        value, point = self.field.minimum(Vector3d.NAN)
        self.assertEqual(value, 0.75)
        self.assertEqual(point, Vector3d.ONE * -0.5)

        region = Region3d(
            Intervald.closed(1., 2.),
            Intervald.closed(1., 2.),
            Intervald.closed(1., 2.))
        self.assertEqual(self.field.minimum(region), 3.)
        value, point = self.field.minimum(region, Vector3d.NAN)
        self.assertEqual(value, 3.)
        self.assertEqual(point, Vector3d.ONE)

        empty = Region3d()
        self.assertTrue(math.isnan(self.field.minimum(empty)))
        value, point = self.field.minimum(empty, Vector3d.ZERO)
        self.assertTrue(math.isnan(value))
        self.assertTrue(math.isnan(point.x()))
        self.assertTrue(math.isnan(point.y()))
        self.assertTrue(math.isnan(point.z()))

    def test_copy_and_string(self):
        expected = "[(x^2 + x + 1) + (y^2 + y + 1) + (z^2 + z + 1)]"
        self.assertEqual(str(self.field), expected)
        self.assertEqual(repr(self.field), expected)
        self.assertEqual(str(copy.copy(self.field)), expected)
        self.assertEqual(str(copy.deepcopy(self.field)), expected)


if __name__ == '__main__':
    unittest.main()
