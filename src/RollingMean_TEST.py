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
from ignition.math import RollingMean


class TestRollingMean(unittest.TestCase):

    def test_rolling_mean(self):
        mean = RollingMean()
        self.assertEqual(0, mean.Count())
        self.assertEqual(10, mean.WindowSize())

        mean.SetWindowSize(4)
        self.assertEqual(4, mean.WindowSize())
        mean.SetWindowSize(0)
        self.assertEqual(4, mean.WindowSize())

        mean.Push(1.0)
        self.assertAlmostEqual(1.0, mean.Mean())
        mean.Push(2.0)
        self.assertAlmostEqual(1.5, mean.Mean())
        mean.Push(3.0)
        self.assertAlmostEqual(2.0, mean.Mean())
        mean.Push(10.0)
        self.assertAlmostEqual(4.0, mean.Mean())
        mean.Push(20.0)
        self.assertAlmostEqual(8.75, mean.Mean())

        mean.Clear()
        self.assertTrue(math.isnan(mean.Mean()))

        mean.Push(100.0)
        mean.Push(200.0)
        mean.Push(300.0)
        self.assertEqual(3, mean.Count())
        mean.SetWindowSize(2)
        self.assertEqual(0, mean.Count())


if __name__ == '__main__':
    unittest.main()
