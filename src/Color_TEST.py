# Copyright (C) 2021 Open Source Robotics Foundation

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
import math
from ignition.math import Color
from ignition.math import Vector3f


class TestColor(unittest.TestCase):

    def test_const_color(self):
        self.assertAlmostEqual(1.0, Color.White.R())
        self.assertAlmostEqual(1.0, Color.White.G())
        self.assertAlmostEqual(1.0, Color.White.B())
        self.assertAlmostEqual(1.0, Color.White.A())

        self.assertAlmostEqual(0.0, Color.Black.R())
        self.assertAlmostEqual(0.0, Color.Black.G())
        self.assertAlmostEqual(0.0, Color.Black.B())
        self.assertAlmostEqual(1.0, Color.Black.A())

        self.assertAlmostEqual(1.0, Color.Red.R())
        self.assertAlmostEqual(0.0, Color.Red.G())
        self.assertAlmostEqual(0.0, Color.Red.B())
        self.assertAlmostEqual(1.0, Color.Red.A())

        self.assertAlmostEqual(0.0, Color.Green.R())
        self.assertAlmostEqual(1.0, Color.Green.G())
        self.assertAlmostEqual(0.0, Color.Green.B())
        self.assertAlmostEqual(1.0, Color.Green.A())

        self.assertAlmostEqual(0.0, Color.Blue.R())
        self.assertAlmostEqual(0.0, Color.Blue.G())
        self.assertAlmostEqual(1.0, Color.Blue.B())
        self.assertAlmostEqual(1.0, Color.Blue.A())

        self.assertAlmostEqual(1.0, Color.Yellow.R())
        self.assertAlmostEqual(1.0, Color.Yellow.G())
        self.assertAlmostEqual(0.0, Color.Yellow.B())
        self.assertAlmostEqual(1.0, Color.Yellow.A())

        self.assertAlmostEqual(1.0, Color.Magenta.R())
        self.assertAlmostEqual(0.0, Color.Magenta.G())
        self.assertAlmostEqual(1.0, Color.Magenta.B())
        self.assertAlmostEqual(1.0, Color.Magenta.A())

        self.assertAlmostEqual(0.0, Color.Cyan.R())
        self.assertAlmostEqual(1.0, Color.Cyan.G())
        self.assertAlmostEqual(1.0, Color.Cyan.B())
        self.assertAlmostEqual(1.0, Color.Cyan.A())

    def test_color(self):
        clr0 = Color()
        self.assertAlmostEqual(0.0, clr0.R())
        self.assertAlmostEqual(0.0, clr0.G())
        self.assertAlmostEqual(0.0, clr0.B())
        self.assertAlmostEqual(1.0, clr0.A())
        self.assertEqual(clr0.AsRGBA(), 255)
        clr0.A(0.0)
        self.assertEqual(clr0.AsRGBA(), 0)

        clr = Color(.1, .2, .3, 1.0)
        self.assertAlmostEqual(0.1, clr.R())
        self.assertAlmostEqual(0.2, clr.G())
        self.assertAlmostEqual(0.3, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.Set(1, 0, 0, 0)
        self.assertAlmostEqual(clr.AsRGBA(), 255 << 24)
        self.assertAlmostEqual(clr.AsBGRA(), 255 << 8)
        self.assertAlmostEqual(clr.AsARGB(), 255 << 16)
        self.assertAlmostEqual(clr.AsABGR(), 255)
        clr0.SetFromRGBA(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.SetFromBGRA(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.SetFromARGB(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.SetFromABGR(255)
        self.assertEqual(clr0, clr)

        clr.Set(0, 1, 0, 0)
        self.assertAlmostEqual(clr.AsRGBA(), 255 << 16)
        self.assertAlmostEqual(clr.AsBGRA(), 255 << 16)
        self.assertAlmostEqual(clr.AsARGB(), 255 << 8)
        self.assertAlmostEqual(clr.AsABGR(), 255 << 8)
        clr0.SetFromRGBA(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.SetFromBGRA(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.SetFromARGB(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.SetFromABGR(255 << 8)
        self.assertEqual(clr0, clr)

        clr.Set(0, 0, 1, 0)
        self.assertAlmostEqual(clr.AsRGBA(), 255 << 8)
        self.assertAlmostEqual(clr.AsBGRA(), 255 << 24)
        self.assertAlmostEqual(clr.AsARGB(), 255)
        self.assertAlmostEqual(clr.AsABGR(), 255 << 16)
        clr0.SetFromRGBA(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.SetFromBGRA(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.SetFromARGB(255)
        self.assertEqual(clr0, clr)
        clr0.SetFromABGR(255 << 16)
        self.assertEqual(clr0, clr)

        clr.Set(0, 0, 0, 1)
        self.assertAlmostEqual(clr.AsRGBA(), 255)
        self.assertAlmostEqual(clr.AsBGRA(), 255)
        self.assertAlmostEqual(clr.AsARGB(), 255 << 24)
        self.assertAlmostEqual(clr.AsABGR(), 255 << 24)
        clr0.SetFromRGBA(255)
        self.assertEqual(clr0, clr)
        clr0.SetFromBGRA(255)
        self.assertEqual(clr0, clr)
        clr0.SetFromARGB(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.SetFromABGR(255 << 24)
        self.assertEqual(clr0, clr)

        clr.Reset()
        self.assertAlmostEqual(0.0, clr.R())
        self.assertAlmostEqual(0.0, clr.G())
        self.assertAlmostEqual(0.0, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.SetFromHSV(0, 0.5, 1.0)
        self.assertAlmostEqual(1.0, clr.R())
        self.assertAlmostEqual(0.5, clr.G())
        self.assertAlmostEqual(0.5, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        self.assertTrue(clr.HSV() == Vector3f(6, 0.5, 1))

        clr.SetFromHSV(60, 0.0, 1.0)
        self.assertAlmostEqual(1.0, clr.R())
        self.assertAlmostEqual(1.0, clr.G())
        self.assertAlmostEqual(1.0, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.SetFromHSV(120, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.R())
        self.assertAlmostEqual(1.0, clr.G())
        self.assertAlmostEqual(0.5, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.SetFromHSV(180, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.R())
        self.assertAlmostEqual(1.0, clr.G())
        self.assertAlmostEqual(1.0, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.SetFromHSV(240, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.R())
        self.assertAlmostEqual(0.5, clr.G())
        self.assertAlmostEqual(1.0, clr.B())
        self.assertAlmostEqual(1.0, clr.A())

        clr.SetFromHSV(300, 0.5, 1.0)
        self.assertAlmostEqual(1.0, clr[0])
        self.assertAlmostEqual(0.5, clr[1])
        self.assertAlmostEqual(1.0, clr[2])
        self.assertAlmostEqual(1.0, clr[3])
        self.assertTrue(math.isnan(clr[4]))

        clr.Set(0.1, 0.2, 0.3, 0.4)
        clr = clr + 0.2
        self.assertTrue(clr == Color(0.3, 0.4, 0.5, 0.6))

        clr.Set(0.1, 0.2, 0.3, 0.4)
        clr += Color(0.2, 0.2, 0.2, 0.2)
        self.assertTrue(clr == Color(0.3, 0.4, 0.5, 0.6))

        clr.Set(0.1, 0.2, 0.3, 0.4)
        clr = clr - 0.1
        self.assertTrue(clr == Color(0.0, 0.1, 0.2, 0.3))

        clr.Set(0.1, 0.2, 0.3, 0.4)
        clr -= Color(0.1, 0.1, 0.1, 0.1)
        self.assertTrue(clr == Color(0.0, 0.1, 0.2, 0.3))

        clr.Set(1.0, 1.0, 1.0, 1.0)
        clr = clr / 1.6
        self.assertTrue(clr == Color(0.625, 0.625, 0.625, 0.625))

        clr.Set(1.0, 1.0, 1.0, 1.0)
        clr /= Color(1.0, 1.0, 1.0, 1.0)
        self.assertTrue(clr == Color(1.0, 1.0, 1.0, 1.0))

        clr.Set(.1, .2, .3, .4)
        clr = clr * .1
        self.assertTrue(clr == Color(0.01, 0.02, 0.03, 0.04))

        clr.Set(.1, .2, .3, .4)
        clr *= Color(0.1, 0.1, 0.1, 0.1)
        self.assertTrue(clr == Color(0.01, 0.02, 0.03, 0.04))

        clr.SetFromYUV(0.5, 0.2, 0.8)
        self.assertAlmostEqual(0.00553, clr.R(), delta=1e-3)
        self.assertAlmostEqual(0.0, clr.G())
        self.assertAlmostEqual(0.9064, clr.B(), delta=1e-3)
        self.assertAlmostEqual(0.04, clr.A())

        self.assertTrue(clr.YUV() == Vector3f(0.104985, 0.95227, 0.429305))

        clr = Color(1.0, 0.0, 0.5, 1.0) + Color(0.1, 0.3, 0.4, 1.0)
        self.assertAlmostEqual(0.00431373, clr.R(), delta=1e-4)
        self.assertAlmostEqual(0.3, clr.G(), delta=1e-4)
        self.assertAlmostEqual(0.9, clr.B(), delta=1e-4)
        self.assertAlmostEqual(1.0, clr.A(), delta=1e-4)

        clr = Color(1.0, 0.0, 0.5, 1.0) - Color(0.1, 0.3, 0.4, 1.0)
        self.assertAlmostEqual(0.9, clr.R(), delta=1e-4)
        self.assertAlmostEqual(0.0, clr.G(), delta=1e-4)
        self.assertAlmostEqual(0.1, clr.B(), delta=1e-4)
        self.assertAlmostEqual(0.0, clr.A(), delta=1e-4)

        clr = Color(0.5, 0.2, 0.4, 0.6) / 2.0
        self.assertAlmostEqual(0.25, clr.R(), delta=1e-4)
        self.assertAlmostEqual(0.1, clr.G(), delta=1e-4)
        self.assertAlmostEqual(0.2, clr.B(), delta=1e-4)
        self.assertAlmostEqual(0.3, clr.A(), delta=1e-4)

    def test_mul(self):
        clr = Color(0.0, 0.01, 0.2, 1.0)
        clr2 = Color(1.0, 0.2, 0.2, 0.0)
        clr3 = clr * clr2

        self.assertAlmostEqual(clr3.R(), 0.0)
        self.assertAlmostEqual(clr3.G(), 0.002)
        self.assertAlmostEqual(clr3.B(), 0.04)
        self.assertAlmostEqual(clr3.A(), 0.0)

    def test_division(self):
        clr = Color(0.0, 0.01, 0.2, 1.0)
        clr2 = clr / 0.2
        self.assertAlmostEqual(clr2.R(), 0.0)
        self.assertAlmostEqual(clr2.G(), 0.05)
        self.assertAlmostEqual(clr2.B(), 1.0)
        self.assertAlmostEqual(clr2.A(), 1.0)

        clr2 = clr / 2.0
        self.assertAlmostEqual(clr2.R(), 0.0)
        self.assertAlmostEqual(clr2.G(), 0.005)
        self.assertAlmostEqual(clr2.B(), 0.1)
        self.assertAlmostEqual(clr2.A(), 0.5)

        clr2.Set(0.0, 0.2, 0.4, 0.5)
        clr3 = clr / clr2
        self.assertAlmostEqual(clr3.R(), 0.0)
        self.assertAlmostEqual(clr3.G(), 0.05)
        self.assertAlmostEqual(clr3.B(), 0.5)
        self.assertAlmostEqual(clr3.A(), 1.0)

        clr.Set(0.0, 0.0, 0.0, 0.0)
        clr2.Set(0.0, 0.0, 0.0, 0.0)
        clr3 = clr / clr2
        self.assertAlmostEqual(clr3.R(), 0.0)
        self.assertAlmostEqual(clr3.G(), 0.0)
        self.assertAlmostEqual(clr3.B(), 0.0)
        self.assertAlmostEqual(clr3.A(), 0.0)

    def test_const_set(self):
        clr = Color(0.1, 0.2, 0.3, 0.4)
        self.assertAlmostEqual(clr.R(), 0.1)
        self.assertAlmostEqual(clr.G(), 0.2)
        self.assertAlmostEqual(clr.B(), 0.3)
        self.assertAlmostEqual(clr.A(), 0.4)

        clr2 = Color()
        clr2.R(0.4)
        clr2.G(0.3)
        clr2.B(0.2)
        clr2.A(0.1)
        self.assertAlmostEqual(clr2.R(), 0.4)
        self.assertAlmostEqual(clr2.G(), 0.3)
        self.assertAlmostEqual(clr2.B(), 0.2)
        self.assertAlmostEqual(clr2.A(), 0.1)

        self.assertTrue(clr2 != clr)

    def test_stream_out(self):
        c = Color(0.1, 0.2, 0.3, 0.5)
        self.assertAlmostEqual(str(c), "0.1 0.2 0.3 0.5")

    def test_HSV(self):
        clr = Color()
        hsv = clr.HSV()
        self.assertAlmostEqual(hsv.X(), -1.0)
        self.assertAlmostEqual(hsv.Y(), 0.0)
        self.assertAlmostEqual(hsv.Z(), 0.0)

        clr.Set(0.1, 0.2, 0.3, 1.0)
        hsv = clr.HSV()
        self.assertAlmostEqual(hsv.X(), 3.5, delta=1e-3)
        self.assertAlmostEqual(hsv.Y(), 0.666667, delta=1e-3)
        self.assertAlmostEqual(hsv.Z(), 0.3, delta=1e-3)

        clr.Set(0.3, 0.2, 0.1, 1.0)
        hsv = clr.HSV()
        self.assertAlmostEqual(hsv.X(), 0.5, delta=1e-3)
        self.assertAlmostEqual(hsv.Y(), 0.666667, delta=1e-3)
        self.assertAlmostEqual(hsv.Z(), 0.3, delta=1e-3)

        clr.SetFromHSV(60, 10, 5)
        self.assertAlmostEqual(clr.R(), 0.0196078, delta=1e-3)
        self.assertAlmostEqual(clr.G(), 0.0196078, delta=1e-3)
        self.assertAlmostEqual(clr.B(), 0.0, delta=1e-3)
        self.assertAlmostEqual(clr.A(), 1.0, delta=1e-3)

        clr.SetFromHSV(360.0, 0.5, 0.6)
        self.assertAlmostEqual(clr.R(), 0.6, delta=1e-3)
        self.assertAlmostEqual(clr.G(), 0.3, delta=1e-3)
        self.assertAlmostEqual(clr.B(), 0.3, delta=1e-3)
        self.assertAlmostEqual(clr.A(), 1.0, delta=1e-3)


if __name__ == '__main__':
    unittest.main()
