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

from gz.math8 import AdditivelySeparableScalarField3d
from gz.math8 import AdditivelySeparableScalarField3f
from gz.math8 import Intervald, Polynomial3d, Polynomial3f, Region3d, Region3f
from gz.math8 import PiecewiseScalarField3d, PiecewiseScalarField3f
from gz.math8 import Vector3d, Vector3f, Vector4d, Vector4f


def constant_field(value):
    zero = Polynomial3d.constant(0.)
    return AdditivelySeparableScalarField3d(
        1., Polynomial3d.constant(value), zero, zero)


def x_field():
    zero = Polynomial3d.constant(0.)
    return AdditivelySeparableScalarField3d(
        1., Polynomial3d(Vector4d(0., 0., 1., 0.)), zero, zero)


def y_field():
    zero = Polynomial3d.constant(0.)
    return AdditivelySeparableScalarField3d(
        1., zero, Polynomial3d(Vector4d(0., 0., 1., 0.)), zero)


class TestPiecewiseScalarField3(unittest.TestCase):

    def test_piece(self):
        region = Region3d.closed(0., 0., 0., 1., 1., 1.)
        piece = PiecewiseScalarField3d.Piece(region, x_field())
        self.assertEqual(piece.region, region)
        self.assertEqual(piece.field(Vector3d(0.5, 0., 0.)), 0.5)

        other = Region3d.open(-1., -1., -1., 0., 0., 0.)
        piece.region = other
        piece.field = y_field()
        self.assertEqual(piece.region, other)
        self.assertEqual(piece.field(Vector3d(0., 0.5, 0.)), 0.5)

    def test_piece_copy_and_string(self):
        piece = PiecewiseScalarField3d.Piece(
            Region3d.closed(0., 0., 0., 1., 1., 1.), x_field())
        expected = "[(x) + (0) + (0)] if (x, y, z) in [0, 1] x [0, 1] x [0, 1]"
        self.assertEqual(str(piece), expected)
        self.assertEqual(repr(piece), expected)

        piece_copy = copy.copy(piece)
        self.assertEqual(str(piece_copy), expected)
        piece_copy.region = Region3d.open(-1., -1., -1., 0., 0., 0.)
        self.assertEqual(piece.region, Region3d.closed(0., 0., 0., 1., 1., 1.))
        self.assertEqual(str(copy.deepcopy(piece)), expected)
        self.assertEqual(
            len(copy.deepcopy([piece, piece])), 2)

    def test_evaluate(self):
        field = PiecewiseScalarField3d()
        self.assertTrue(math.isnan(field(Vector3d.ZERO)))

        field = PiecewiseScalarField3d.throughout(x_field())
        self.assertEqual(field(Vector3d.ZERO), 0.)
        self.assertEqual(field(Vector3d(0.5, 0.5, 0.5)), 0.5)
        self.assertEqual(field.evaluate(Vector3d.ONE), 1.)
        self.assertEqual(field.evaluate(Vector3d.UNIT_X), 1.)

        # An empty region never contains a point.
        # Note: the constructor logs an error to stderr for empty regions.
        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d.open(0., 0., 0., 0., 0., 0.), constant_field(1.)),
            PiecewiseScalarField3d.Piece(
                Region3d.closed(0., 0., 0., 1., 1., 1.), x_field())])
        self.assertEqual(field(Vector3d.ZERO), 0.)

        # The first region takes precedence when regions overlap.
        # Note: the constructor logs an error to stderr for overlaps.
        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d.closed(0., 0., 0., 0., 0., 0.), constant_field(1.)),
            PiecewiseScalarField3d.Piece(
                Region3d.closed(0., 0., 0., 1., 1., 1.), x_field())])
        self.assertEqual(field(Vector3d.ZERO), 1.)

        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d.open(0., 0., 0., 1., 1., 1.), x_field()),
            PiecewiseScalarField3d.Piece(
                Region3d.open(-1., -1., -1., 0., 0., 0.), y_field())])
        self.assertEqual(field(Vector3d(0.5, 0.25, 0.5)), 0.5)
        self.assertEqual(field(Vector3d(-0.5, -0.25, -0.5)), -0.25)
        self.assertTrue(math.isnan(field(Vector3d(0.5, -0.25, 0.5))))
        self.assertTrue(math.isnan(field(Vector3d(-0.5, 0.25, -0.5))))

        zero = Polynomial3f.constant(0.)
        field = PiecewiseScalarField3f.throughout(
            AdditivelySeparableScalarField3f(
                1., Polynomial3f(Vector4f(0., 0., 1., 0.)), zero, zero))
        self.assertEqual(field(Vector3f.ONE), 1.)
        field = PiecewiseScalarField3f([
            PiecewiseScalarField3f.Piece(
                Region3f.closed(0., 0., 0., 1., 1., 1.),
                AdditivelySeparableScalarField3f(1., zero, zero, zero))])
        self.assertTrue(math.isnan(field(Vector3f(2., 2., 2.))))

    def test_minimum(self):
        field = PiecewiseScalarField3d()
        self.assertTrue(math.isnan(field.minimum()))
        value, point = field.minimum(Vector3d.ZERO)
        self.assertTrue(math.isnan(value))
        self.assertTrue(math.isnan(point.x()))
        self.assertTrue(math.isnan(point.y()))
        self.assertTrue(math.isnan(point.z()))

        # Only empty-region pieces: minimum is undefined
        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d.open(0., 0., 0., 0., 0., 0.), constant_field(1.))])
        self.assertTrue(math.isnan(field.minimum()))
        value, point = field.minimum(Vector3d.ZERO)
        self.assertTrue(math.isnan(value))
        self.assertTrue(math.isnan(point.x()))
        self.assertTrue(math.isnan(point.y()))
        self.assertTrue(math.isnan(point.z()))

        field = PiecewiseScalarField3d.throughout(
            AdditivelySeparableScalarField3d(
                1.,
                Polynomial3d.constant(0.),
                Polynomial3d.constant(1.),
                Polynomial3d.constant(0.)))
        self.assertEqual(field.minimum(), 1.)
        value, point = field.minimum(Vector3d.NAN)
        self.assertEqual(value, 1.)
        self.assertFalse(math.isnan(point.x()))
        self.assertFalse(math.isnan(point.y()))
        self.assertFalse(math.isnan(point.z()))

        zero = Polynomial3d.constant(0.)
        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d.open(0., 0., 0., 1., 1., 1.),
                AdditivelySeparableScalarField3d(
                    1., Polynomial3d(Vector4d(0., 1., 0., 0.)), zero, zero)),
            PiecewiseScalarField3d.Piece(
                Region3d.open(-1., -1., -1., 0., 0., 0.),
                AdditivelySeparableScalarField3d(
                    1., zero, Polynomial3d(Vector4d(0., 1., 0., 1.)), zero))])
        self.assertEqual(field.minimum(), 0.)
        value, point = field.minimum(Vector3d.NAN)
        self.assertEqual(value, 0.)
        self.assertEqual(point, Vector3d.ZERO)

        # The input point is not modified
        start = Vector3d(1., 2., 3.)
        field.minimum(start)
        self.assertEqual(start, Vector3d(1., 2., 3.))

        zerof = Polynomial3f.constant(0.)
        field = PiecewiseScalarField3f()
        self.assertTrue(math.isnan(field.minimum()))
        field = PiecewiseScalarField3f.throughout(
            AdditivelySeparableScalarField3f(
                1., zerof, Polynomial3f.constant(1.), zerof))
        self.assertEqual(field.minimum(), 1.)
        start = Vector3f(1., 2., 3.)
        value, point = field.minimum(start)
        self.assertEqual(value, 1.)
        self.assertFalse(math.isnan(point.x()))
        self.assertEqual(start, Vector3f(1., 2., 3.))

    def test_copy_and_string(self):
        self.assertEqual(str(PiecewiseScalarField3d()), "undefined")

        inf = math.inf
        unbounded = Intervald.open(-inf, inf)
        field = PiecewiseScalarField3d([
            PiecewiseScalarField3d.Piece(
                Region3d(unbounded, unbounded, Intervald.open(-inf, 0.)),
                AdditivelySeparableScalarField3d(
                    1.,
                    Polynomial3d(Vector4d(0., 1., 0., 0.)),
                    Polynomial3d(Vector4d(0., 1., 0., 0.)),
                    Polynomial3d(Vector4d(0., 1., 0., 0.)))),
            PiecewiseScalarField3d.Piece(
                Region3d(unbounded, unbounded, Intervald.left_closed(0., inf)),
                AdditivelySeparableScalarField3d(
                    -1.,
                    Polynomial3d(Vector4d(0., 0., 1., 0.)),
                    Polynomial3d(Vector4d(0., 0., 1., 0.)),
                    Polynomial3d(Vector4d(0., 0., 1., 0.))))])
        expected = (
            "[(x^2) + (y^2) + (z^2)] if (x, y, z) "
            "in (-inf, inf) x (-inf, inf) x (-inf, 0); "
            "-[(x) + (y) + (z)] if (x, y, z) "
            "in (-inf, inf) x (-inf, inf) x [0, inf)")
        self.assertEqual(str(field), expected)
        self.assertEqual(repr(field), expected)
        self.assertEqual(str(copy.copy(field)), expected)
        self.assertEqual(str(copy.deepcopy(field)), expected)

        self.assertEqual(str(PiecewiseScalarField3f()), "undefined")
        zerof = Polynomial3f.constant(0.)
        field = PiecewiseScalarField3f.throughout(
            AdditivelySeparableScalarField3f(
                1., Polynomial3f(Vector4f(0., 0., 1., 0.)), zerof, zerof))
        expected = (
            "[(x) + (0) + (0)] if (x, y, z) "
            "in (-inf, inf) x (-inf, inf) x (-inf, inf)")
        self.assertEqual(str(field), expected)
        self.assertEqual(repr(field), expected)
        self.assertEqual(str(copy.copy(field)), expected)
        self.assertEqual(str(copy.deepcopy(field)), expected)


if __name__ == '__main__':
    unittest.main()
