# 
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
  #
#

import unittest
import math 
from ignition.math import Vector4d

class TestVector3(unittest.TestCase):

  def test_construction(self):
    v = Vector4d()

    # ::Distance, ::Length()
    v.Set(1, 2, 3, 4)
    self.assertEqual(v.Length(), v.Distance(Vector4d.Zero))

    # ::operator/ vector4
    v.Set(4, 4, 4, 4)
    v = v / Vector4d(1, 2, 2, 4)
    self.assertEqual(v, Vector4d(4, 2, 2, 1))

    # ::operator / double
    v = v / 2
    self.assertEqual(v, Vector4d(2, 1, 1, 0.5))

    # ::operator * vector4
    v = v * Vector4d(2, 3, 3, 4)
    self.assertEqual(v, Vector4d(4, 3, 3, 2))

    # operator /=
    v.Set(1, 2, 2, 4)
    v /= Vector4d(1, 4, 8, 4)
    self.assertEqual(v, Vector4d(1, 0.5, 0.25, 1))

    # operator *=
    v.Set(1, 2, 2, 4)
    v *= Vector4d(2, 0.5, 0.25, 0.1)
    self.assertEqual(v, Vector4d(2, 1, 0.5, 0.4))

    # Test the static defines.
    self.assertEqual(Vector4d.Zero,
        Vector4d(0, 0, 0, 0))

    self.assertEqual(Vector4d.One,
        Vector4d(1, 1, 1, 1))

  def test_distance(self):
    vec1 = Vector4d(0, 0, 0, 0)
    vec2 = Vector4d(1, 2, 3, 4)

    dist = vec1.Distance(vec2)
    self.assertTrue(abs(dist - 5.47722557505) < 1e-6)
  
  def test_squared_length(self):
    vec1 = Vector4d(0, 0, 0, 0)
    vec2 = Vector4d(1, 2, 3, 4)

    sum1 = vec1.SquaredLength()
    sum2 = vec2.SquaredLength()

    self.assertEqual(sum1, 0)
    self.assertEqual(sum2, 30)

  def test_length(self):
    # Zero vector
    self.assertEqual(Vector4d.Zero.Length(), 0.0)
    self.assertEqual(Vector4d.Zero.SquaredLength(), 0.0)

    # One vector
    self.assertTrue(abs(Vector4d.One.Length() - math.sqrt(4.0)) < 1e-10)

    self.assertEqual(Vector4d.One.SquaredLength(), 4.0)

    # Arbitrary vector
    v = Vector4d(0.1, -4.2, 2.5, -1.2)
    self.assertTrue(abs(v.Length() - 5.03388517946) < 1e-10)

    self.assertTrue(abs(v.SquaredLength() - 25.34) < 1e-10)
  
  def test_normalize(self):
    vec1 = Vector4d(0, 0, 0, 0)
    vec2 = Vector4d(1, 2, 3, 4)

    vec3 = vec1
    vec3.Normalize()
    self.assertEqual(vec3, vec1)
    self.assertEqual(vec1, Vector4d.Zero)

    vec3 = vec2
    vec2.Normalize()
    self.assertTrue(vec2.Equal(Vector4d(0.182575, 0.365150, 0.547725, 0.730300), 1e-5))
  
  def test_add(self):
    vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)
    vec2 = Vector4d(1.1, 2.2, 3.4, 4.3)

    vec3 = vec1
    vec3 += vec2

    self.assertEqual(vec1 + vec2, Vector4d(1.2, 2.4, 3.8, 5.1))
    self.assertEqual(vec3, Vector4d(1.2, 2.4, 3.8, 5.1))

    # Addition with zeros

    # Scalar left and right
    self.assertEqual(vec1 + 0, vec1)

    # Vector left and right
    self.assertEqual(Vector4d.Zero + vec1, vec1)
    self.assertEqual(vec1 + Vector4d.Zero, vec1)

    # Addition assignment
    vec4 = vec1
    vec4 += 0
    self.assertEqual(vec4, vec1)
    vec4 += Vector4d.Zero
    self.assertEqual(vec4, vec1)

    # Add non-trivial scalar values left and right
    self.assertEqual(vec1 + 2.5, Vector4d(2.6, 2.7, 2.9, 3.3))

    vec1 = vec4
    vec4 += 2.5
    self.assertEqual(vec4, Vector4d(2.6, 2.7, 2.9, 3.3))
  
  def test_sub(self):
    vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)
    vec2 = Vector4d(1.1, 2.2, 3.4, 4.3)

    vec3 = vec2
    vec3 -= vec1

    self.assertEqual(vec2 - vec1, Vector4d(1.0, 2.0, 3.0, 3.5))
    self.assertEqual(vec3, Vector4d(1.0, 2.0, 3.0, 3.5))

    # Subtraction with zeros

    # Scalar left and right
    self.assertEqual(vec1 - 0, vec1)

    # Vector left and right
    self.assertEqual(Vector4d.Zero - vec1, -vec1)
    self.assertEqual(vec1 - Vector4d.Zero, vec1)

    # Subtraction assignment
    vec4 = vec1
    vec4 -= 0
    self.assertEqual(vec4, vec1)
    vec4 -= Vector4d.Zero
    self.assertEqual(vec4, vec1)

    # Subtract non-trivial scalar values left and right
    self.assertEqual(vec1 - 2.5, -Vector4d(2.4, 2.3, 2.1, 1.7))

    vec4 = vec1
    vec4 -= 2.5
    self.assertEqual(vec4, -Vector4d(2.4, 2.3, 2.1, 1.7))
  

  def test_divide(self):
    vec1 = Vector4d(0.1, 0.2, 0.4, 0.8)

    vec3 = vec1 / 2.0
    self.assertEqual(vec3, Vector4d(0.05, 0.1, 0.2, 0.4))

    vec3 /= 4.0
    self.assertEqual(vec3, Vector4d(0.0125, 0.025, 0.05, 0.1))
  
  def test_multiply(self):
    v = Vector4d(0.1, 0.2, 0.3, 0.4)

    vec3 = v * 2.0
    self.assertEqual(vec3, Vector4d(0.2, 0.4, 0.6, 0.8))

    vec3 *= 4.0
    self.assertEqual(vec3, Vector4d(0.8, 1.6, 2.4, 3.2))

    # Multiply by zero

    # Scalar left and right
    self.assertEqual(v * 0, Vector4d.Zero)

    # Element-wise vector multiplication
    self.assertEqual(v * Vector4d.Zero, Vector4d.Zero)

    # Multiply by one

    # Scalar left and right
    self.assertEqual(v * 1, v)

    # Element-wise vector multiplication
    self.assertEqual(v * Vector4d.One, v,)

    # Multiply by non-trivial scalar value

    scalar = 2.5
    expect = Vector4d(0.25, 0.5, 0.75, 1.0)
    self.assertEqual(v * scalar, expect)

    # Multiply by itself element-wise
    self.assertEqual(v*v, Vector4d(0.01, 0.04, 0.09, 0.16))

  def test_not_equal(self):
    vec1 = Vector4d(0.1, 0.2, 0.3, 0.4)
    vec2 = Vector4d(0.2, 0.2, 0.3, 0.4)
    vec3 = Vector4d(0.1, 0.2, 0.3, 0.4)

    self.assertTrue(vec1 != vec2)
    self.assertTrue(not(vec1 != vec3))

  def test_equal(self):
    self.assertTrue(not Vector4d.Zero.Equal(
    Vector4d.One, 1e-6))
    self.assertTrue(not Vector4d.Zero.Equal(
    Vector4d.One, 1e-3))
    self.assertTrue(not Vector4d.Zero.Equal(
    Vector4d.One, 1e-1))

    self.assertTrue(Vector4d.Zero.Equal(
        Vector4d.One, 1))
    self.assertTrue(Vector4d.Zero.Equal(
        Vector4d.One, 1.1))
  
  def test_finite(self):
    vec1 = Vector4d(0.1, 0.2, 0.3, 0.4)
    self.assertTrue(vec1.IsFinite())


if __name__ == '__main__':
  unittest.main()
