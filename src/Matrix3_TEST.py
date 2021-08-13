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
from ignition.math import Matrix3d
from ignition.math import Vector3d


class TestMatrix3(unittest.TestCase):

    def test_matrix3d(self):
        matrix = Matrix3d()
        self.assertAlmostEqual(matrix, Matrix3d(0, 0, 0, 0, 0, 0, 0, 0, 0))

        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9))

        matrix1 = Matrix3d(matrix)
        self.assertAlmostEqual(matrix1, Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9))

        matrix = Matrix3d()
        matrix.Axes(Vector3d(1, 1, 1), Vector3d(2, 2, 2),
                    Vector3d(3, 3, 3))
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 3, 1, 2, 3, 1, 2, 3))

        matrix.Axis(Vector3d(1, 1, 1), math.pi)
        self.assertAlmostEqual(matrix, Matrix3d(1, 2, 2, 2, 1, 2, 2, 2, 1))

        matrix.Col(0, Vector3d(3, 4, 5))
        self.assertAlmostEqual(matrix, Matrix3d(3, 2, 2, 4, 1, 2, 5, 2, 1))

        matrix.Col(3, Vector3d(1, 1, 1))
        self.assertAlmostEqual(matrix, Matrix3d(3, 2, 1, 4, 1, 1, 5, 2, 1))

    def test_sub(self):
        matZero = Matrix3d.Zero
        matIdent = Matrix3d.Identity

        mat = matIdent - matZero
        self.assertAlmostEqual(mat, matIdent)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(10, 20, 30,
                        40, 50, 60,
                        70, 80, 90)

        mat = matB - matA
        self.assertAlmostEqual(mat, Matrix3d(9, 18, 27,
                                             36, 45, 54,
                                             63, 72, 81))

    def test_add(self):

        matZero = Matrix3d.Zero
        matIdent = Matrix3d.Identity

        mat = matIdent + matZero
        self.assertAlmostEqual(mat, matIdent)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(10, 20, 30,
                        40, 50, 60,
                        70, 80, 90)

        mat = matB + matA
        self.assertAlmostEqual(mat, Matrix3d(11, 22, 33,
                                             44, 55, 66,
                                             77, 88, 99))

    def test_mul(self):
        matZero = Matrix3d.Zero
        matIdent = Matrix3d.Identity

        mat = matIdent * matZero
        self.assertAlmostEqual(mat, matZero)

        matA = Matrix3d(1, 2, 3,
                        4, 5, 6,
                        7, 8, 9)
        matB = Matrix3d(10, 20, 30,
                        40, 50, 60,
                        70, 80, 90)

        mat = matB * matA
        self.assertAlmostEqual(mat, Matrix3d(300, 360, 420,
                                             660, 810, 960,
                                             1020, 1260, 1500))

        mat = matB * matA
        self.assertAlmostEqual(mat, Matrix3d(300, 360, 420,
                                             660, 810, 960,
                                             1020, 1260, 1500))

        mat = mat * 2.0
        self.assertAlmostEqual(mat, Matrix3d(600, 720, 840,
                                             1320, 1620, 1920,
                                             2040, 2520, 3000))

    def test_vector3_mul(self):
        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)

        # Scalar
        self.assertAlmostEqual(Matrix3d.Zero, matrix * 0)

        # Vector3.Zero
        self.assertAlmostEqual(Vector3d.Zero, matrix * Vector3d.Zero)

        # Matrix3.Zero
        self.assertAlmostEqual(Matrix3d.Zero, matrix * Matrix3d.Zero)
        self.assertAlmostEqual(Matrix3d.Zero, Matrix3d.Zero * matrix)

        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)

        # scalar 1.0
        self.assertAlmostEqual(matrix, matrix * 1.0)

        # Vector3.Unit
        # right multiply
        self.assertAlmostEqual(Vector3d(matrix(0, 0), matrix(1, 0),
                                        matrix(2, 0)), matrix * Vector3d.UnitX)
        self.assertAlmostEqual(Vector3d(matrix(0, 1), matrix(1, 1),
                                        matrix(2, 1)), matrix * Vector3d.UnitY)
        self.assertAlmostEqual(Vector3d(matrix(0, 2), matrix(1, 2),
                                        matrix(2, 2)), matrix * Vector3d.UnitZ)

        # Matrix3.Identity
        self.assertAlmostEqual(matrix, matrix * Matrix3d.Identity)
        self.assertAlmostEqual(matrix, Matrix3d.Identity * matrix)

        # Multiply arbitrary matrix by itself
        matrix = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        matrix2 = Matrix3d(30,  36,  42,
                           66,  81,  96,
                           102, 126, 150)

        self.assertAlmostEqual(matrix * matrix, matrix2)

    def test_not_equal(self):
        matrix1 = Matrix3d()
        matrix2 = Matrix3d()

        self.assertTrue(matrix1 == matrix2)
        self.assertFalse(matrix1 != matrix2)

        matrix1 = Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9)
        matrix2 = Matrix3d(matrix1)

        self.assertFalse(matrix1 != matrix1)

        matrix2 = Matrix3d(1.00001, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertTrue(matrix1 != matrix2)

        matrix2 = Matrix3d(1.000001, 2, 3, 4, 5, 6, 7, 8, 9)
        self.assertFalse(matrix1 != matrix2)

    def test_equal_tolerance(self):
        self.assertFalse(Matrix3d.Zero.Equal(Matrix3d.Identity, 1e-6))
        self.assertFalse(Matrix3d.Zero.Equal(Matrix3d.Identity, 1e-3))
        self.assertFalse(Matrix3d.Zero.Equal(Matrix3d.Identity, 1e-1))
        self.assertTrue(Matrix3d.Zero.Equal(Matrix3d.Identity, 1))
        self.assertTrue(Matrix3d.Zero.Equal(Matrix3d.Identity, 1.1))

    def test_inverse(self):
        self.assertAlmostEqual(Matrix3d.Identity, Matrix3d.Identity.Inverse())

        # Matrix multiplied by its inverse results in the identity matrix
        matrix1 = Matrix3d(-2, 4, 0, 0.1, 9, 55, -7, 1, 26)
        matrix2 = matrix1.Inverse()
        self.assertAlmostEqual(matrix1 * matrix2, Matrix3d.Identity)
        self.assertAlmostEqual(matrix2 * matrix1, Matrix3d.Identity)

        # Inverse of inverse results in the same matrix
        self.assertAlmostEqual((matrix1.Inverse()).Inverse(), matrix1)

        # Invert multiplication by scalar
        scalar = 2.5
        self.assertAlmostEqual((matrix1 * scalar).Inverse(),
                               matrix1.Inverse() * (1.0/scalar))

    def test_determinant(self):
        # |Zero matrix| = 0.0
        self.assertAlmostEqual(0.0, Matrix3d.Zero.Determinant())

        # |Identity matrix| = 1.0
        self.assertAlmostEqual(1.0, Matrix3d.Identity.Determinant())

        # Determinant of arbitrary matrix
        m = Matrix3d(-2, 4, 0, 0.1, 9, 55, -7, 1, 26)
        self.assertAlmostEqual(-1908.4, m.Determinant())

    def test_transpose(self):
        # Transpose of zero matrix is itself
        self.assertAlmostEqual(Matrix3d.Zero, Matrix3d.Zero.Transposed())

        # Transpose of identity matrix is itself
        self.assertAlmostEqual(Matrix3d.Identity,
                               Matrix3d.Identity.Transposed())

        # Matrix and expected transpose
        m = Matrix3d(-2, 4, 0,
                     0.1, 9, 55,
                     -7, 1, 26)
        mT = Matrix3d(-2, 0.1, -7,
                      4,   9, 1,
                      0,  55, 26)
        self.assertNotEqual(m, mT)
        self.assertAlmostEqual(m.Transposed(), mT)
        self.assertAlmostEqual(m.Determinant(), m.Transposed().Determinant())

        mT.Transpose()
        self.assertAlmostEqual(m, mT)

    def test_from2axes(self):
        v1 = Vector3d(1.0, 0.0, 0.0)
        v2 = Vector3d(0.0, 1.0, 0.0)

        m1 = Matrix3d()
        m1.From2Axes(v1, v2)

        m2 = Matrix3d()
        m2.From2Axes(v2, v1)

        m1Correct = Matrix3d(0, -1, 0,
                             1, 0, 0,
                             0, 0, 1)
        m2Correct = Matrix3d(m1Correct)
        m2Correct.Transpose()

        self.assertNotEqual(m1, m2)
        self.assertAlmostEqual(m1Correct, m1)
        self.assertAlmostEqual(m2Correct, m2)
        self.assertAlmostEqual(Matrix3d.Identity, m1 * m2)
        self.assertAlmostEqual(v2, m1 * v1)
        self.assertAlmostEqual(v1, m2 * v2)

        # rotation about 45 degrees
        v1.Set(1.0, 0.0, 0.0)
        v2.Set(1.0, 1.0, 0.0)
        m2.From2Axes(v1, v2)
        # m1 is 90 degrees rotation
        self.assertAlmostEqual(m1, m2*m2)

        # with non-unit vectors
        v1.Set(0.5, 0.5, 0)
        v2.Set(-0.5, 0.5, 0)

        m1.From2Axes(v1, v2)
        m2.From2Axes(v2, v1)

        self.assertNotEqual(m1, m2)
        self.assertAlmostEqual(m1Correct, m1)
        self.assertAlmostEqual(m2Correct, m2)
        self.assertAlmostEqual(Matrix3d.Identity, m1 * m2)
        self.assertAlmostEqual(v2, m1 * v1)
        self.assertAlmostEqual(v1, m2 * v2)

        # For zero-length vectors, a unit matrix is returned
        v1.Set(0, 0, 0)
        v2.Set(-0.5, 0.5, 0)
        m1.From2Axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.Identity, m1)

        # For zero-length vectors, a unit matrix is returned
        v1.Set(-0.5, 0.5, 0)
        v2.Set(0, 0, 0)
        m1.From2Axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.Identity, m1)

        # Parallel vectors
        v1.Set(1, 0, 0)
        v2.Set(2, 0, 0)
        m1.From2Axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.Identity, m1)

        # Opposite vectors
        v1.Set(1, 0, 0)
        v2.Set(-2, 0, 0)
        m1.From2Axes(v1, v2)
        self.assertAlmostEqual(Matrix3d.Zero - Matrix3d.Identity, m1)


if __name__ == '__main__':
    unittest.main()
