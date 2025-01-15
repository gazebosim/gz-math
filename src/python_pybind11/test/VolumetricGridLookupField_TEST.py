# Copyright (C) 2025 Open Source Robotics Foundation

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

from gz.math8 import Vector3d, VolumetricGridLookupFieldd


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

        # Test inside point (should return 8 interpolators??)
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


if __name__ == "__main__":
    unittest.main()
