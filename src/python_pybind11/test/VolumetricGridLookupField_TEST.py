# Copyright (C) 2026 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from gz.math import InterpolationPoint3Dd, Vector3d, VolumetricGridLookupFieldd


class TestVolumetricGridLookupField(unittest.TestCase):
    def test_interpolation_box_eight_points(self):
        # Create cloud points as in the C++ test
        cloud = [
            Vector3d(0, 0, 0),
            Vector3d(0, 0, 1),
            Vector3d(0, 1, 0),
            Vector3d(0, 1, 1),
            Vector3d(1, 0, 0),
            Vector3d(1, 0, 1),
            Vector3d(1, 1, 0),
            Vector3d(1, 1, 1),
        ]

        # Create lookup field
        field = VolumetricGridLookupFieldd(cloud)

        # Test inside point (should return 8 interpolators)
        pos = Vector3d(0.5, 0.5, 0.5)
        interpolators = field.get_interpolators(pos)
        self.assertEqual(len(interpolators), 8)

        # Test outside point (should return 0 interpolators)
        pos = Vector3d(-0.5, -0.5, -0.5)
        interpolators = field.get_interpolators(pos)
        self.assertEqual(len(interpolators), 0)

        # Test point on plane (should return 4 interpolators)
        pos = Vector3d(0.5, 0.5, 0)
        interpolators = field.get_interpolators(pos)
        self.assertEqual(len(interpolators), 4)

        # Test point on edge (should return 2 interpolators)
        pos = Vector3d(0.5, 0, 0)
        interpolators = field.get_interpolators(pos)
        self.assertEqual(len(interpolators), 2)

    def test_trilinear_interpolation(self):
        # Create cloud points
        cloud = [
            Vector3d(0, 0, 0),
            Vector3d(0, 0, 1),
            Vector3d(0, 1, 0),
            Vector3d(0, 1, 1),
            Vector3d(1, 0, 0),
            Vector3d(1, 0, 1),
            Vector3d(1, 1, 0),
            Vector3d(1, 1, 1),
        ]

        values = [0, 0, 0, 0, 1, 1, 1, 1]
        field = VolumetricGridLookupFieldd(cloud)

        # Test inside point
        pos = Vector3d(0.5, 0.5, 0.5)
        value = field.estimate_value_using_trilinear(pos, values)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value, 0.5, places=3)

        # Test outside point
        pos = Vector3d(-0.5, -0.5, -0.5)
        value = field.estimate_value_using_trilinear(pos, values)
        self.assertIsNone(value)

        # Test point on plane
        pos = Vector3d(0, 0.5, 0.5)
        value = field.estimate_value_using_trilinear(pos, values)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value, 0.0, places=3)

        # Test point on vertex
        pos = Vector3d(0, 0, 0)
        value = field.estimate_value_using_trilinear(pos, values)
        self.assertIsNotNone(value)
        self.assertAlmostEqual(value, 0.0, places=3)

    def test_trilinear_interpolation_with_interpolators(self):
        # Create cloud points
        cloud = [
            Vector3d(0, 0, 0),
            Vector3d(0, 0, 1),
            Vector3d(0, 1, 0),
            Vector3d(0, 1, 1),
            Vector3d(1, 0, 0),
            Vector3d(1, 0, 1),
            Vector3d(1, 1, 0),
            Vector3d(1, 1, 1),
        ]

        field = VolumetricGridLookupFieldd(cloud)

        # Compute the interpolators once and reuse them for several
        # value arrays sampled at the same point
        pos = Vector3d(0.5, 0.5, 0.5)
        interpolators = field.get_interpolators(pos)

        values_x = [0, 0, 0, 0, 1, 1, 1, 1]
        value = field.estimate_value_using_trilinear(
            interpolators, pos, values_x)
        self.assertAlmostEqual(value, 0.5, places=3)

        values_z = [0, 1, 0, 1, 0, 1, 0, 1]
        value = field.estimate_value_using_trilinear(
            interpolators, pos, values_z)
        self.assertAlmostEqual(value, 0.5, places=3)

        # Outside point yields empty interpolators and no value
        pos = Vector3d(-0.5, -0.5, -0.5)
        interpolators = field.get_interpolators(pos)
        value = field.estimate_value_using_trilinear(
            interpolators, pos, values_x)
        self.assertIsNone(value)

    def test_bounds(self):
        cloud = [
            Vector3d(0, 0, 0),
            Vector3d(1, 2, 3),
        ]

        field = VolumetricGridLookupFieldd(cloud)
        lower, upper = field.bounds()
        self.assertEqual(lower, Vector3d(0, 0, 0))
        self.assertEqual(upper, Vector3d(1, 2, 3))

    def test_interpolation_point(self):
        point = InterpolationPoint3Dd()
        self.assertIsNone(point.index)

        point = InterpolationPoint3Dd(position=Vector3d(1, 2, 3), index=4)
        self.assertEqual(point.position, Vector3d(1, 2, 3))
        self.assertEqual(point.index, 4)

        point.index = None
        self.assertIsNone(point.index)
        self.assertIn("index=None", str(point))

        cloud = [
            Vector3d(0, 0, 0),
            Vector3d(0, 0, 1),
            Vector3d(0, 1, 0),
            Vector3d(0, 1, 1),
            Vector3d(1, 0, 0),
            Vector3d(1, 0, 1),
            Vector3d(1, 1, 0),
            Vector3d(1, 1, 1),
        ]

        field = VolumetricGridLookupFieldd(cloud)
        interpolators = field.get_interpolators(Vector3d(0, 0, 0))
        self.assertEqual(len(interpolators), 1)
        self.assertEqual(interpolators[0].position, Vector3d(0, 0, 0))
        self.assertEqual(interpolators[0].index, 0)


if __name__ == "__main__":
    unittest.main()
