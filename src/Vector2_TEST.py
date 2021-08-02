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
from ignition.math import Vector2d


class Vector2Test(unittest.TestCase):

    def test_construction(self):
        v = Vector2d()
        self.assertAlmostEqual(0.0, v.X())
        self.assertAlmostEqual(0.0, v.Y())

        vec = Vector2d(1, 0)
        self.assertEqual(vec.X(), 1)
        self.assertEqual(vec.Y(), 0)

        vec2 = Vector2d(vec)
        self.assertEqual(vec2, vec)

        # Copy
        vec3 = vec
        self.assertEqual(vec3, vec)

        # Inequality
        vec4 = Vector2d()
        self.assertNotEqual(vec, vec4)

    def test_vector2(self):
        v = Vector2d(1, 2)

        # Distance
        self.assertAlmostEqual(2.236, v.Distance(Vector2d()), delta=1e-2)

        # Normalize
        v.Normalize()
        self.assertTrue(v.Equal(Vector2d(0.447214, 0.894427), 1e-4))

        # Set
        v.Set(4, 5)
        self.assertTrue(v.Equal(Vector2d(4, 5), 1e-4))

        # Operator GetAbs
        v.Set(-1, -2)
        self.assertTrue(v.Abs().Equal(Vector2d(1, 2), 1e-4))

        # _eq_
        v = Vector2d(6, 7)
        self.assertTrue(v.Equal(Vector2d(6, 7), 1e-4))

        # _add_
        v = v + Vector2d(1, 2)
        self.assertTrue(v.Equal(Vector2d(7, 9), 1e-4))

        v += Vector2d(5, 6)
        self.assertTrue(v.Equal(Vector2d(12, 15), 1e-4))

        # __sub__
        v = v - Vector2d(2, 4)
        self.assertTrue(v.Equal(Vector2d(10, 11), 1e-4))

        v.Set(2, 4)
        v -= Vector2d(1, 6)
        self.assertTrue(v.Equal(Vector2d(1, -2), 1e-4))

        # __truediv__
        v.Set(10, 6)
        v = v / Vector2d(2, 3)
        self.assertTrue(v.Equal(Vector2d(5, 2), 1e-4))

        v.Set(10, 6)
        v /= Vector2d(2, 3)
        self.assertTrue(v.Equal(Vector2d(5, 2), 1e-4))

        # __truediv__ int
        v.Set(10, 6)
        v = v / 2
        self.assertTrue(v.Equal(Vector2d(5, 3), 1e-4))

        v.Set(10, 6)
        v /= 2
        self.assertTrue(v.Equal(Vector2d(5, 3), 1e-4))

        # __mul__
        v.Set(10, 6)
        v = v * Vector2d(2, 4)
        self.assertTrue(v.Equal(Vector2d(20, 24), 1e-4))

        v.Set(10, 6)
        v *= Vector2d(2, 4)
        self.assertTrue(v.Equal(Vector2d(20, 24), 1e-4))

        # __mul__ int
        v.Set(10, 6)
        v = v * 2
        self.assertTrue(v.Equal(Vector2d(20, 12), 1e-4))

        v.Set(10, 6)
        v *= 2
        self.assertTrue(v.Equal(Vector2d(20, 12), 1e-4))

        # IsFinite
        self.assertTrue(v.IsFinite())

    def test_max(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(0.3, 0.5)
        vec3 = Vector2d(0.4, 0.2)

        self.assertAlmostEqual(vec1.Max(), 0.2)
        self.assertAlmostEqual(vec3.Max(), 0.4)

        vec1.Max(vec2)
        self.assertAlmostEqual(vec1, Vector2d(0.3, 0.5))

        vec1.Max(vec3)
        self.assertAlmostEqual(vec1, Vector2d(0.4, 0.5))

    def test_min(self):
        vec1 = Vector2d(0.3, 0.5)
        vec2 = Vector2d(0.1, 0.2)
        vec3 = Vector2d(0.05, 0.1)

        self.assertAlmostEqual(vec1.Min(), 0.3)
        self.assertAlmostEqual(vec3.Min(), 0.05)

        vec1.Min(vec2)
        self.assertAlmostEqual(vec1, Vector2d(0.1, 0.2))

        vec1.Min(vec3)
        self.assertAlmostEqual(vec1, Vector2d(0.05, 0.1))

    def test_equal_tolerance(self):
        # Test Equal function with specified tolerance
        self.assertFalse(Vector2d.Zero.Equal(Vector2d.One, 1e-6))
        self.assertFalse(Vector2d.Zero.Equal(Vector2d.One, 1e-3))
        self.assertFalse(Vector2d.Zero.Equal(Vector2d.One, 1e-1))
        self.assertTrue(Vector2d.Zero.Equal(Vector2d.One, 1))
        self.assertTrue(Vector2d.Zero.Equal(Vector2d.One, 1.1))

    def test_dot(self):
        v = Vector2d(1, 2)
        self.assertAlmostEqual(v.Dot(Vector2d(3, 4)), 11.0)
        self.assertAlmostEqual(v.Dot(Vector2d(0, 0)), 0.0)
        self.assertAlmostEqual(v.Dot(Vector2d(1, 0)), 1.0)
        self.assertAlmostEqual(v.Dot(Vector2d(0, 1)), 2.0)

    def test_correct(self):
        vec1 = Vector2d(0, float("nan"))
        vec2 = Vector2d(float("inf"), -1)
        vec3 = Vector2d(10, -2)

        vec1.Correct()
        vec2.Correct()
        vec3.Correct()

        self.assertAlmostEqual(vec1, Vector2d(0, 0))
        self.assertAlmostEqual(vec2, Vector2d(0, -1))
        self.assertAlmostEqual(vec3, Vector2d(10, -2))

    def test_abs_dot(self):
        v = Vector2d(1, -2)

        self.assertAlmostEqual(v.AbsDot(Vector2d(3, 4)), 11.0)
        self.assertAlmostEqual(v.AbsDot(Vector2d(0, 0)), 0.0)
        self.assertAlmostEqual(v.AbsDot(Vector2d(1, 0)), 1.0)
        self.assertAlmostEqual(v.AbsDot(Vector2d(0, 1)), 2.0)

    def test_add(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(1.1, 2.2)

        vec3 = vec1
        vec3 += vec2

        self.assertAlmostEqual(vec1 + vec2, Vector2d(1.2, 2.4))
        self.assertAlmostEqual(vec3, Vector2d(1.2, 2.4))

        # Add zero
        # Scalar right
        self.assertEqual(vec1 + 0, vec1)

        # Vector left and right
        self.assertAlmostEqual(Vector2d.Zero + vec1, vec1)
        self.assertAlmostEqual(vec1 + Vector2d.Zero, vec1)

        # Addition assigment
        vec4 = Vector2d(vec1)
        vec4 += 0
        self.assertEqual(vec4, vec1)
        vec4 += Vector2d.Zero
        self.assertAlmostEqual(vec4, vec1)

        # Add non-trivial scalar values left and right
        self.assertEqual(vec1 + 2.5, Vector2d(2.6, 2.7))

        vec1 = vec4
        vec4 += 2.5
        self.assertEqual(vec4, Vector2d(2.6, 2.7))

    def test_sub(self):
        vec1 = Vector2d(0.1, 0.2)
        vec2 = Vector2d(1.1, 2.2)

        vec3 = vec2
        vec3 -= vec1

        self.assertAlmostEqual(vec2 - vec1, Vector2d(1.0, 2.0))
        self.assertAlmostEqual(vec3, Vector2d(1.0, 2.0))

        # Subtraction with zeros
        # Scalar right
        self.assertEqual(vec1 - 0, vec1)

        # Vector left and right
        self.assertAlmostEqual(Vector2d.Zero - vec1, -vec1)
        self.assertAlmostEqual(vec1 - Vector2d.Zero, vec1)

        # Subtraction assignment
        vec4 = Vector2d(vec1)
        vec4 -= 0
        self.assertEqual(vec4, vec1)
        vec4 -= Vector2d.Zero
        self.assertAlmostEqual(vec4, vec1)

        # Subtract non-trivial scalar values left and right
        self.assertEqual(vec1 - 2.5, -Vector2d(2.4, 2.3))

        vec4 = vec1
        vec4 -= 2.5
        self.assertEqual(vec4, -Vector2d(2.4, 2.3))

    def test_multiply(self):
        v = Vector2d(0.1, -4.2)

        vec2 = v * 2.0
        self.assertEqual(vec2, Vector2d(0.2, -8.4))

        vec2 *= 4.0
        self.assertEqual(vec2, Vector2d(0.8, -33.6))

        # Multiply by zero
        # Scalar right
        self.assertEqual(v * 0, Vector2d.Zero)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector2d.Zero, Vector2d.Zero)

        # Multiply by one
        # Scalar right
        self.assertEqual(v * 1, v)

        # Element-wise vector multiplication
        self.assertEqual(v * Vector2d.One, v)

        # Multiply by non-trivial scalar value
        scalar = 2.5
        expect = Vector2d(0.25, -10.5)
        self.assertEqual(v * scalar, expect)

        # Multiply by itself element-wise
        v.Set(0.1, 0.5)
        self.assertAlmostEqual(v * v, Vector2d(0.01, 0.25))

    def test_lenght(self):
        # Zero vector
        self.assertAlmostEqual(Vector2d.Zero.Length(), 0.0)
        self.assertAlmostEqual(Vector2d.Zero.SquaredLength(), 0.0)

        # One vector
        self.assertAlmostEqual(Vector2d.One.Length(),
                               math.sqrt(2), delta=1e-10)
        self.assertAlmostEqual(Vector2d.One.SquaredLength(), 2.0)

        # Arbitrary vector
        v = Vector2d(0.1, -4.2)
        self.assertAlmostEqual(v.Length(), 4.20119030752, delta=1e-10)
        self.assertAlmostEqual(v.SquaredLength(), 17.65)

        # Integer vector
        v = Vector2d(3, 4)
        self.assertAlmostEqual(v.Length(), 5)
        self.assertAlmostEqual(v.SquaredLength(), 25)


if __name__ == '__main__':
    unittest.main()
