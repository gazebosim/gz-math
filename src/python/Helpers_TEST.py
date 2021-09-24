# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import unittest

from ignition.math import (IGN_BOX_VOLUME, IGN_BOX_VOLUME_V, IGN_CYLINDER_VOLUME,
                           IGN_PI, IGN_SPHERE_VOLUME, NAN_I, Vector3d, equal, fixnan,
                           greaterOrNearEqual, is_even, is_odd, is_power_of_two, isnan,
                           lessOrNearEqual, max, mean, min,
                           parse_float, parse_int, precision, round_up_multiple,
                           round_up_power_of_two, signum, sort2, sort3, variance)


class TestHelpers(unittest.TestCase):

    def test_helpers(self):
        self.assertEqual(12345, parse_int('12345'))
        self.assertEqual(-12345, parse_int('-12345'))
        self.assertEqual(-12345, parse_int('    -12345'))
        self.assertEqual(0, parse_int('    '))
        self.assertEqual(23, parse_int('23ab67'))

        self.assertEqual(NAN_I, parse_int(''))
        self.assertEqual(NAN_I, parse_int('?'))
        self.assertEqual(NAN_I, parse_int('ab23ab67'))

        self.assertEqual(12.345, parse_float('12.345'))
        self.assertEqual(-12.345, parse_float('-12.345'))
        self.assertEqual(-12.345, parse_float('    -12.345'))
        self.assertEqual(0.0, parse_float('    '))
        self.assertTrue(equal(123.45, parse_float('1.2345e2'), 1e-2))
        self.assertTrue(equal(123.45, parse_float('1.2345e+2'), 1e-2))
        self.assertTrue(equal(123.45, parse_float('1.2345e+002'), 1e-2))
        self.assertTrue(equal(.012345, parse_float('1.2345e-2'), 1e-2))
        self.assertTrue(equal(.012345, parse_float('1.2345e-002'), 1e-2))
        self.assertTrue(equal(1.2345, parse_float('1.2345e+'), 1e-2))
        self.assertTrue(equal(1.2345, parse_float('1.2345e-'), 1e-2))
        self.assertTrue(lessOrNearEqual(1.0, 2.0, 1e-2))
        self.assertTrue(lessOrNearEqual(1.0, 1.0 - 9e-3, 1e-2))
        self.assertFalse(lessOrNearEqual(1.0, 1.0 - 1.1e-2, 1e-2))
        self.assertTrue(greaterOrNearEqual(1.0, 0.5, 1e-2))
        self.assertTrue(greaterOrNearEqual(1.0, 1.0 + 9e-3, 1e-2))
        self.assertFalse(greaterOrNearEqual(1.0, 1.0 + 1.1e-2, 1e-2))
        self.assertEqual(1.2345, parse_float('1.2345e+0'))
        self.assertEqual(23.0, parse_float('23ab67'))

        self.assertTrue(isnan(parse_float('')))
        self.assertTrue(isnan(parse_float('?')))
        self.assertTrue(isnan(parse_float('ab23ab67')))

        self.assertEqual(1, round_up_power_of_two(0))
        self.assertEqual(1, round_up_power_of_two(1))
        self.assertEqual(2, round_up_power_of_two(2))
        self.assertEqual(2048, round_up_power_of_two(1025))

    def test_precision(self):
        # Test Helpers::precision
        self.assertEqual(0, precision(0.0, 1))
        self.assertAlmostEqual(0.1, precision(0.1, 1), 6)
        self.assertAlmostEqual(0.1, precision(0.14, 1), 6)
        self.assertAlmostEqual(0.2, precision(0.15, 1), 6)
        self.assertAlmostEqual(0.15, precision(0.15, 2), 6)

        self.assertAlmostEqual(1, precision(1.4, 0), 6)
        self.assertAlmostEqual(0, precision(0, 0), 6)

    def test_is_power_of_two(self):
        # Test Helpers::is_power_of_two
        self.assertFalse(is_power_of_two(0))
        self.assertFalse(is_power_of_two(3))

        self.assertTrue(is_power_of_two(1))

        self.assertTrue(is_power_of_two(2))
        self.assertTrue(is_power_of_two(4))

    # MSVC report errors on division by zero
    # Test Helpers::fixnan functions
    def test_fix_nan(self):
        self.assertEqual(fixnan(42.0), 42.0)
        self.assertEqual(fixnan(-42.0), -42.0)

    def test_even(self):
        i = 1

        self.assertFalse(is_even(i))

        i = -1

        self.assertFalse(is_even(i))

        i = 4

        self.assertTrue(is_even(i))

        i = -2

        self.assertTrue(is_even(i))

        i = 0

        self.assertTrue(is_even(i))

    # Odd test
    def test_odd(self):
        i = 1

        self.assertTrue(is_odd(i))

        i = -1

        self.assertTrue(is_odd(i))

        i = 4

        self.assertFalse(is_odd(i))

        i = -2

        self.assertFalse(is_odd(i))

        i = 0

        self.assertFalse(is_odd(i))

    # Signum test
    def test_signum(self):
        i = 1
        f = 1.0

        self.assertEqual(1, signum(i))
        self.assertEqual(1, signum(f))

        i = 2
        f = 2.0

        self.assertEqual(1, signum(i))
        self.assertEqual(1, signum(f))

        i = 0
        f = 0.0

        self.assertEqual(0, signum(i))
        self.assertEqual(0, signum(f))

        i = -1
        f = -1.0

        self.assertEqual(-1, signum(i))
        self.assertEqual(-1, signum(f))

        i = -2
        f = -2.0

        self.assertEqual(-1, signum(i))
        self.assertEqual(-1, signum(f))

        f = -2.5

        self.assertEqual(-1, signum(f))

        f = 2.5

        self.assertEqual(1, signum(f))

        f = 1e-10

        self.assertEqual(1, signum(f))

    def test_calculation(self):
        self.assertEqual(2, max([0, 1, 2]))
        self.assertEqual(0, min([0, 1, 2]))
        self.assertEqual(1, mean([0, 1, 2]))
        self.assertEqual(0, variance([0, 1, 2]))

    def test_sort(self):
        a = 2
        b = -1
        a, b = sort2(a, b)
        self.assertLess(a, b)

        a = 0
        b = 1
        a, b = sort2(a, b)
        self.assertLess(a, b)

        a = 2
        b = -1
        c = 0
        a, b, c = sort3(a, b, c)
        self.assertLess(a, b)
        self.assertLess(b, c)

        a = 2
        b = 1
        a, b = sort2(a, b)
        self.assertLess(a, b)

        a = 2
        b = 1
        c = 0
        a, b, c = sort3(a, b, c)
        self.assertLess(a, b)
        self.assertLess(b, c)

        a = 0
        b = 1
        c = 2
        a, b, c = sort3(a, b, c)
        self.assertLess(a, b)
        self.assertLess(b, c)

        a = 2.1
        b = -1.1e-1
        a, b = sort2(a, b)
        self.assertLess(a, b)

        a = 34.5
        b = -1.34
        c = 0.194
        a, b, c = sort3(a, b, c)
        self.assertLess(a, b)
        self.assertLess(b, c)

        a = 2.1
        b = -1.1e-1
        a, b = sort2(a, b)
        self.assertLess(a, b)

        a = 34.5
        b = -1.34
        c = 0.194
        a, b, c = sort3(a, b, c)
        self.assertLess(a, b)
        self.assertLess(b, c)

    def test_volume(self):
        self.assertEqual(IGN_SPHERE_VOLUME(1.0), 4.0*IGN_PI*math.pow(1, 3)/3.0)
        self.assertEqual(IGN_SPHERE_VOLUME(0.1), 4.0*IGN_PI*math.pow(.1, 3)/3.0)
        self.assertEqual(IGN_SPHERE_VOLUME(-1.1), 4.0*IGN_PI*math.pow(-1.1, 3)/3.0)

        self.assertEqual(IGN_CYLINDER_VOLUME(0.5, 2.0), 2 * IGN_PI * math.pow(.5, 2))
        self.assertEqual(IGN_CYLINDER_VOLUME(1, -1), -1 * IGN_PI * math.pow(1, 2))

        self.assertEqual(IGN_BOX_VOLUME(1, 2, 3), 1 * 2 * 3)
        self.assertEqual(IGN_BOX_VOLUME(.1, .2, .3),
                         IGN_BOX_VOLUME_V(Vector3d(0.1, 0.2, 0.3)))

    def test_round_up_multiple(self):
        self.assertEqual(0, round_up_multiple(0, 0))
        self.assertEqual(12, round_up_multiple(12, 0))

        self.assertEqual(1, round_up_multiple(1, 1))
        self.assertEqual(100, round_up_multiple(100, 10))
        self.assertEqual(48, round_up_multiple(48, 12))

        self.assertEqual(4, round_up_multiple(3, 2))
        self.assertEqual(23, round_up_multiple(3, 23))
        self.assertEqual(6, round_up_multiple(6, 3))
        self.assertEqual(9, round_up_multiple(7, 3))

        self.assertEqual(-8, round_up_multiple(-9, 2))
        self.assertEqual(-6, round_up_multiple(-7, 3))

        self.assertEqual(0, round_up_multiple(-1, 2))

        self.assertEqual(2, round_up_multiple(2, -2))
        self.assertEqual(0, round_up_multiple(0, -2))
        self.assertEqual(-2, round_up_multiple(-2, -2))


if __name__ == '__main__':
    unittest.main()
