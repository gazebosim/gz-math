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
from ignition.math import Matrix4d
from ignition.math import Pose3d
from ignition.math import Quaterniond
from ignition.math import Vector3d


class TestMatrix4(unittest.TestCase):
    def test_construct(self):
        mat = Matrix4d()

        for i in range(5):
            for j in range(5):
                self.assertAlmostEqual(mat(i, j), 0.0)

        mat2 = Matrix4d(mat)
        for i in range(5):
            for j in range(5):
                self.assertAlmostEqual(mat2(i, j), 0.0)

        self.assertTrue(mat2 == mat)

        mat3 = Matrix4d(0.0, 1.0, 2.0, 3.0,
                        4.0, 5.0, 6.0, 7.0,
                        8.0, 9.0, 10.0, 11.0,
                        12.0, 13.0, 14.0, 15.0)

        mat4 = Matrix4d(mat3)
        self.assertAlmostEqual(mat4, mat3)

        self.assertAlmostEqual(mat3(0, 0), 0.0)
        self.assertAlmostEqual(mat3(0, 1), 1.0)
        self.assertAlmostEqual(mat3(0, 2), 2.0)
        self.assertAlmostEqual(mat3(0, 3), 3.0)
        self.assertAlmostEqual(mat3(1, 0), 4.0)
        self.assertAlmostEqual(mat3(1, 1), 5.0)
        self.assertAlmostEqual(mat3(1, 2), 6.0)
        self.assertAlmostEqual(mat3(1, 3), 7.0)
        self.assertAlmostEqual(mat3(2, 0), 8.0)
        self.assertAlmostEqual(mat3(2, 1), 9.0)
        self.assertAlmostEqual(mat3(2, 2), 10.0)
        self.assertAlmostEqual(mat3(2, 3), 11.0)
        self.assertAlmostEqual(mat3(3, 0), 12.0)
        self.assertAlmostEqual(mat3(3, 1), 13.0)
        self.assertAlmostEqual(mat3(3, 2), 14.0)
        self.assertAlmostEqual(mat3(3, 3), 15.0)

    def test_construct_from_pose3(self):
        trans = Vector3d(1, 2, 3)
        qt = Quaterniond(0.1, 0.2, 0.3)
        pose = Pose3d(trans, qt)
        mat = Matrix4d(pose)

        self.assertAlmostEqual(pose, mat.Pose())
        self.assertAlmostEqual(trans, mat.Translation())
        self.assertAlmostEqual(qt, mat.Rotation())
        self.assertAlmostEqual(pose.Inverse(), mat.Inverse().Pose())
        # ensure inverses multiply to identity
        self.assertAlmostEqual(mat.Inverse() * mat, Matrix4d.Identity)
        self.assertAlmostEqual(mat * mat.Inverse(), Matrix4d.Identity)
        self.assertAlmostEqual(pose.Inverse() * pose, Pose3d.Zero)
        self.assertAlmostEqual(pose * pose.Inverse(), Pose3d.Zero)

        # repeat test with *=
        m = Matrix4d(Matrix4d.Identity)
        m *= mat
        self.assertAlmostEqual(m, mat)
        m *= mat.Inverse()
        self.assertAlmostEqual(m, Matrix4d.Identity)

        p = Pose3d()
        p *= pose
        self.assertAlmostEqual(p, pose)
        p *= pose.Inverse()
        self.assertAlmostEqual(p, Pose3d.Zero)

        # Zero values
        trans = Vector3d(0, 0, 0)
        qt = Quaterniond(0, 0, 0)
        pose = Pose3d(trans, qt)
        mat = Matrix4d(pose)

        self.assertAlmostEqual(pose, mat.Pose())
        self.assertAlmostEqual(trans, mat.Translation())
        self.assertAlmostEqual(qt, mat.Rotation())
        self.assertAlmostEqual(pose.Inverse(), mat.Inverse().Pose())

        # Rotate pitch by pi/2 so yaw coincides with roll causing a gimbal lock
        trans = Vector3d(3, 2, 1)
        qt = Quaterniond(0, math.pi/2, 0)
        pose = Pose3d(trans, qt)
        mat = Matrix4d(pose)

        self.assertAlmostEqual(pose, mat.Pose())
        self.assertAlmostEqual(trans, mat.Translation())
        self.assertAlmostEqual(qt, mat.Rotation())
        self.assertAlmostEqual(pose.Inverse(), mat.Inverse().Pose())

        # setup a ZXZ rotation to ensure non-commutative rotations
        pose1 = Pose3d(1, -2, 3, 0, 0, math.pi/4)
        pose2 = Pose3d(0, 1, -1, -math.pi/4, 0, 0)
        pose3 = Pose3d(-1, 0, 0, 0, 0, -math.pi/4)

        m1 = Matrix4d(pose1)
        m2 = Matrix4d(pose2)
        m3 = Matrix4d(pose3)

        # ensure rotations are not commutative
        self.assertNotEqual(m1 * m2 * m3, m3 * m2 * m1)

        # ensure pose multiplication order matches matrix order
        self.assertAlmostEqual(m1 * m2 * m3, Matrix4d(pose1 * pose2 * pose3))
        self.assertAlmostEqual(m3 * m2 * m1, Matrix4d(pose3 * pose2 * pose1))

        # repeat test with *=
        m = Matrix4d(Matrix4d.Identity)
        p = Pose3d()

        m *= m1
        p *= pose1
        self.assertAlmostEqual(m, m1)
        self.assertAlmostEqual(p, pose1)
        self.assertAlmostEqual(m, Matrix4d(p))

        m *= m2
        p *= pose2
        self.assertAlmostEqual(m, m1 * m2)
        self.assertAlmostEqual(p, pose1 * pose2)
        self.assertAlmostEqual(m, Matrix4d(p))

        m *= m3
        p *= pose3
        self.assertAlmostEqual(m, m1 * m2 * m3)
        self.assertAlmostEqual(p, pose1 * pose2 * pose3)
        self.assertAlmostEqual(m, Matrix4d(p))

    def test_scale(self):
        mat = Matrix4d()
        mat2 = Matrix4d()

        mat.Scale(Vector3d(1, 2, 3))
        mat2.Scale(1, 2, 3)

        self.assertAlmostEqual(mat, mat2)

        self.assertAlmostEqual(mat(0, 0), 1.0)
        self.assertAlmostEqual(mat(1, 1), 2.0)
        self.assertAlmostEqual(mat(2, 2), 3.0)
        self.assertAlmostEqual(mat(3, 3), 1.0)

        self.assertAlmostEqual(mat2(0, 0), 1.0)
        self.assertAlmostEqual(mat2(1, 1), 2.0)
        self.assertAlmostEqual(mat2(2, 2), 3.0)
        self.assertAlmostEqual(mat2(3, 3), 1.0)

        self.assertAlmostEqual(mat.Scale(), mat2.Scale())
        self.assertAlmostEqual(mat.Scale(), Vector3d(1, 2, 3))

        for i in range(0, 4):
            for j in range(0, 4):
                if i != j:
                    self.assertAlmostEqual(mat(i, j), 0.0)
                    self.assertAlmostEqual(mat2(i, j), 0.0)
                elif i == 3 and j == 3:
                    self.assertAlmostEqual(mat(i, j), 1.0)
                    self.assertAlmostEqual(mat2(i, j), 1.0)

    def test_multiply_vect(self):
        mat = Matrix4d()
        vec = Vector3d(-1.2, 2.3, 10.5)

        self.assertAlmostEqual(mat * vec, Vector3d(0.0, 0.0, 0.0))

        mat = Matrix4d(Matrix4d.Identity)
        self.assertAlmostEqual(mat * vec, vec)

    def test_multiply_mat(self):
        mat = Matrix4d(0, -1, -2, -3,
                       1, 0, -1, -2,
                       2, 1, 0, -1,
                       3, 2, 1, 0)
        mat1 = Matrix4d(0, 1, 2, 3,
                        1, 2, 3, 4,
                        2, 3, 4, 5,
                        3, 4, 5, 6)

        mat3 = Matrix4d(-14, -20, -26, -32,
                        -8, -10, -12, -14,
                        -2, 0, 2, 4,
                        4, 10, 16, 22)

        mat2 = mat * mat1

        self.assertAlmostEqual(mat2, mat3)

        mat4 = mat
        mat4 *= mat1
        self.assertAlmostEqual(mat2, mat4)

    def test_inverse(self):
        mat = Matrix4d(2, 3, 1, 5,
                       1, 0, 3, 1,
                       0, 2, -3, 2,
                       0, 2, 3, 1)

        mat1 = mat.Inverse()
        self.assertAlmostEqual(mat1, Matrix4d(18, -35, -28, 1,
                                              9, -18, -14, 1,
                                              -2, 4, 3, 0,
                                              -12, 24, 19, -1))

    def test_get_pose3(self):
        mat = Matrix4d(2, 3, 1, 5,
                       1, 0, 3, 1,
                       0, 2, -3, 2,
                       0, 2, 3, 1)
        pose = mat.Pose()

        self.assertAlmostEqual(pose, Pose3d(5, 1, 2,
                                            -0.204124, 1.22474, 0.816497, 0.204124))

    def test_translation(self):
        mat = Matrix4d()
        mat2 = Matrix4d()

        mat.SetTranslation(Vector3d(1, 2, 3))
        mat2.SetTranslation(1, 2, 3)

        self.assertEqual(mat, mat2)

        self.assertAlmostEqual(mat(0, 3), 1.0)
        self.assertAlmostEqual(mat(1, 3), 2.0)
        self.assertAlmostEqual(mat(2, 3), 3.0)

        self.assertAlmostEqual(mat2(0, 3), 1.0)
        self.assertAlmostEqual(mat2(1, 3), 2.0)
        self.assertAlmostEqual(mat2(2, 3), 3.0)

        self.assertEqual(mat.Translation(), mat2.Translation())
        self.assertEqual(mat.Translation(), Vector3d(1, 2, 3))

        for i in range(0, 4):
            for j in range(0, 2):
                self.assertAlmostEqual(mat(i, j), 0.0)
                self.assertAlmostEqual(mat2(i, j), 0.0)

        self.assertAlmostEqual(mat(3, 3), 0.0)
        self.assertAlmostEqual(mat2(3, 3), 0.0)

    def test_rotation_diag_zero(self):
        mat = Matrix4d(0, 0.2, 0.3, 0.4,
                       0.5, 0, 0.7, 0.8,
                       0.9, 1.0, 0, 1.2,
                       1.3, 1.4, 1.5, 1.0)

        quat = mat.Rotation()
        self.assertAlmostEqual(quat.X(), 0.5, delta=1e-6)
        self.assertAlmostEqual(quat.Y(), 0.35, delta=1e-6)
        self.assertAlmostEqual(quat.Z(), 0.6, delta=1e-6)
        self.assertAlmostEqual(quat.W(), 0.15, delta=1e-6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(1.5708, -1.11977, 1.5708))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(-1.5708, 4.26136, -1.5708))

    def test_rotation_diag_less_zero(self):
        mat = Matrix4d(-0.1, 0.2, 0.3, 0.4,
                        0.5, 0, 0.7, 0.8,
                        0.9, 1.0, 0, 1.2,
                        1.3, 1.4, 1.5, 1.0)

        quat = mat.Rotation()
        self.assertAlmostEqual(quat.X(), 0.333712, delta=1e-6)
        self.assertAlmostEqual(quat.Y(), 0.524404, delta=1e-6)
        self.assertAlmostEqual(quat.Z(), 0.810443, delta=1e-6)
        self.assertAlmostEqual(quat.W(), -0.286039, delta=1e-6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(1.5708, -1.11977, 1.76819))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(-1.5708, 4.26136, -1.3734))

        mat = Matrix4d(-0.1, 0.2, 0.3, 0.4,
                       0.5, -0.2, 0.7, 0.8,
                       0.9, 1.0, 0.0, 1.2,
                       1.3, 1.4, 1.5, 1.0)

        quat = mat.Rotation()
        self.assertAlmostEqual(quat.X(), 0.526235, delta=1e-6)
        self.assertAlmostEqual(quat.Y(), 0.745499, delta=1e-6)
        self.assertAlmostEqual(quat.Z(), 0.570088, delta=1e-6)
        self.assertAlmostEqual(quat.W(), 0.131559, delta=1e-6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(1.5708, -1.11977, 1.76819))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(-1.5708, 4.26136, -1.3734))

    def test_rotation(self):
        mat = Matrix4d(0.1, 0.2, 0.3, 0.4,
                       0.5, 0.6, 0.7, 0.8,
                       0.9, 1.0, 1.1, 1.2,
                       1.3, 1.4, 1.5, 1.6)

        quat = mat.Rotation()
        self.assertAlmostEqual(quat.X(), 0.0896421, delta=1e-6)
        self.assertAlmostEqual(quat.Y(), -0.179284, delta=1e-6)
        self.assertAlmostEqual(quat.Z(), 0.0896421, delta=1e-6)
        self.assertAlmostEqual(quat.W(), 0.83666, delta=1e-6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(0.737815, -1.11977, 1.3734))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(-2.40378, 4.26136, -1.76819))

    def test_euler_rotation2(self):
        mat = Matrix4d(0.1, 0.2, 0.3, 0.4,
                       0.5, 0.6, 0.7, 0.8,
                       1.9, 1.2, 1.1, 1.2,
                       1.3, 1.4, 1.5, 1.6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(-2.55359, -1.5708, 0))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(-2.55359, -1.5708, 0))

        mat = Matrix4d(0.1, 0.2, 0.3, 0.4,
                       0.5, 0.6, 0.7, 0.8,
                       -1.2, 1.2, 1.1, 1.2,
                       1.3, 1.4, 1.5, 1.6)

        euler = mat.EulerRotation(True)
        self.assertAlmostEqual(euler, Vector3d(0.588003, 1.5708, 0))

        euler = mat.EulerRotation(False)
        self.assertAlmostEqual(euler, Vector3d(0.588003, 1.5708, 0))

    def test_affine_transform(self):
        mat = Matrix4d(Matrix4d.Zero)
        vec = Vector3d(1, 2, 3)

        v = Vector3d()

        self.assertFalse(mat.TransformAffine(vec, v))

        mat = Matrix4d(Matrix4d.Identity)
        self.assertTrue(mat.TransformAffine(vec, v))

    def test_stream_out(self):
        matA = Matrix4d(1, 2, 3, 4,
                        5, 6, 7, 8,
                        9, 10, 11, 12,
                        13, 14, 15, 16)

        self.assertEqual(str(matA), "1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16")

    def test_not_equal(self):
        matrix1 = Matrix4d()
        matrix2 = Matrix4d()
        self.assertTrue(matrix1 == matrix2)
        self.assertFalse(matrix1 != matrix2)

        matrix1 = Matrix4d(1, 2, 3, 4,
                           5, 6, 7, 8,
                           9, 10, 11, 12,
                           13, 14, 15, 16)
        matrix2 = Matrix4d(matrix1)

        self.assertFalse(matrix1 != matrix2)

        matrix2 = Matrix4d(1.00001, 2, 3, 4,
                           5, 6, 7, 8,
                           9, 10, 11, 12,
                           13, 14, 15, 16)
        self.assertTrue(matrix1 != matrix2)

        matrix2 = Matrix4d(1.0000001, 2, 3, 4,
                           5, 6, 7, 8,
                           9, 10, 11, 12,
                           13, 14, 15, 16)
        self.assertFalse(matrix1 != matrix2)

    def test_equal_tolerance(self):
        self.assertFalse(Matrix4d.Zero.Equal(Matrix4d.Identity, 1e-6))
        self.assertFalse(Matrix4d.Zero.Equal(Matrix4d.Identity, 1e-3))
        self.assertFalse(Matrix4d.Zero.Equal(Matrix4d.Identity, 1e-1))
        self.assertTrue(Matrix4d.Zero.Equal(Matrix4d.Identity, 1))
        self.assertTrue(Matrix4d.Zero.Equal(Matrix4d.Identity, 1.1))

    def test_determinant(self):
        # |Zero matrix| = 0.0
        self.assertAlmostEqual(0.0, Matrix4d.Zero.Determinant())

        # |Identity matrix| = 1.0
        self.assertAlmostEqual(1.0, Matrix4d.Identity.Determinant())

        # Determinant of arbitrary matrix
        m = Matrix4d(2, 3, 0.1, -5,
                     1, 0, 3.2, 1,
                     0, 2, -3, 2.1,
                     0, 2, 3.2, 1)
        self.assertAlmostEqual(129.82, m.Determinant())

    def test_transpose(self):
        # Transpose of zero matrix is itself
        self.assertAlmostEqual(Matrix4d.Zero, Matrix4d.Zero.Transposed())

        # Transpose of identity matrix is itself
        self.assertAlmostEqual(Matrix4d.Identity,
                               Matrix4d.Identity.Transposed())

        # Matrix and expected transpose
        m = Matrix4d(-2, 4, 0, -3.5,
                     0.1, 9, 55, 1.2,
                     -7, 1, 26, 11.5,
                     .2, 3, -5, -0.1)
        mT = Matrix4d(-2, 0.1, -7, .2,
                      4, 9, 1, 3,
                      0, 55, 26, -5,
                      -3.5, 1.2, 11.5, -0.1)
        self.assertNotEqual(m, mT)
        self.assertEqual(m.Transposed(), mT)
        self.assertAlmostEqual(m.Determinant(), m.Transposed().Determinant())

        mT.Transpose()
        self.assertEqual(m, mT)

    def test_look_at(self):
        self.assertAlmostEqual(Matrix4d.LookAt(-Vector3d.UnitX,
                               Vector3d.Zero).Pose(),
                               Pose3d(-1, 0, 0, 0, 0, 0))

        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(3, 2, 0),
                               Vector3d(0, 2, 0)).Pose(),
                               Pose3d(3, 2, 0, 0, 0, math.pi))

        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(1, 6, 1),
                               Vector3d.One).Pose(),
                               Pose3d(1, 6, 1, 0, 0, -math.pi/2))

        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(-1, -1, 0),
                               Vector3d(1, 1, 0)).Pose(),
                               Pose3d(-1, -1, 0, 0, 0, math.pi/4))

        # Default up is Z
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(0.1, -5, 222),
                               Vector3d(999, -0.6, 0)),
                               Matrix4d.LookAt(Vector3d(0.1, -5, 222),
                               Vector3d(999, -0.6, 0),
                               Vector3d.UnitZ))

        # up == zero, default up = +Z
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(1.23, 456, 0.7),
                               Vector3d(0, 8.9, -10),
                               Vector3d.Zero),
                               Matrix4d.LookAt(Vector3d(1.23, 456, 0.7),
                               Vector3d(0, 8.9, -10)))

        # up == +X, default up = +Z
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(0.25, 9, -5),
                               Vector3d(-6, 0, 0.4),
                               Vector3d.UnitX),
                               Matrix4d.LookAt(Vector3d(0.25, 9, -5),
                               Vector3d(-6, 0, 0.4)))

        # up == -X, default up = +Z
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(0, 0, 0.2),
                               Vector3d(-8, 0, -6),
                               -Vector3d.UnitX),
                               Matrix4d.LookAt(Vector3d(0, 0, 0.2),
                               Vector3d(-8, 0, -6)))

        # eye == target, default direction = +X
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d.One,
                               Vector3d.One),
                               Matrix4d.LookAt(Vector3d.One,
                               Vector3d(1.0001, 1, 1)))

        # Not possible to keep _up on +Z
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d(-1, 0, 10),
                               Vector3d(-1, 0, 0)),
                               Matrix4d.LookAt(Vector3d(-1, 0, 10),
                               Vector3d(-1, 0, 0),
                               -Vector3d.UnitX))

        # Different ups
        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d.One,
                               Vector3d(0, 1, 1),
                               Vector3d.UnitY).Pose(),
                               Pose3d(1, 1, 1, math.pi/2, 0, math.pi))

        self.assertAlmostEqual(Matrix4d.LookAt(Vector3d.One,
                               Vector3d(0, 1, 1),
                               Vector3d(0, 1, 1)).Pose(),
                               Pose3d(1, 1, 1, math.pi/4, 0, math.pi))


if __name__ == '__main__':
    unittest.main()
