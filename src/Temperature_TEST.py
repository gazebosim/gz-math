# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from ignition.math import Temperature


class TestTemperature(unittest.TestCase):

    def test_temperatur_constructor(self):
        temp = Temperature()
        self.assertAlmostEqual(temp.Kelvin(), 0.0, 1e-6)

        temp2 = Temperature(1.1)
        self.assertAlmostEqual(temp2.Kelvin(), 1.1, 1e-6)

        temp3 = Temperature(temp2)
        self.assertAlmostEqual(temp3.Kelvin(), 1.1, delta=1e-6)
        self.assertAlmostEqual(temp3.Celsius(), -272.05, delta=1e-6)

        self.assertTrue(temp2 == temp3)
        self.assertTrue(temp2 == 1.1)
        self.assertTrue(temp2 != temp)
        self.assertTrue(temp2 != 1.2)

        self.assertTrue(temp < temp2)
        self.assertTrue(temp < 10.0)
        self.assertTrue(temp <= temp2)
        self.assertTrue(temp <= 0.0)
        self.assertTrue(temp <= 0.1)

        self.assertFalse(temp > temp2)
        self.assertFalse(temp > 80.0)
        self.assertFalse(temp >= temp2)
        self.assertFalse(temp >= 0.1)
        self.assertTrue(temp >= 0.0)

    def test_temperatur_conversions(self):
        self.assertAlmostEqual(Temperature.KelvinToCelsius(0), -273.15,
                               delta=1e-6)
        self.assertAlmostEqual(Temperature.KelvinToFahrenheit(300), 80.33,
                               delta=1e-6)
        self.assertAlmostEqual(Temperature.CelsiusToFahrenheit(20.0), 68.0,
                               delta=1e-6)
        self.assertAlmostEqual(Temperature.CelsiusToKelvin(10.0), 283.15,
                               delta=1e-6)
        self.assertAlmostEqual(Temperature.FahrenheitToCelsius(-40.0),
                               Temperature.CelsiusToFahrenheit(-40.0), 1e-6)
        self.assertAlmostEqual(Temperature.FahrenheitToKelvin(60.0), 288.7055,
                               delta=1e-3)

    def test_temperatur_mutators_accessors(self):
        temp = Temperature()
        self.assertAlmostEqual(temp.Kelvin(), 0.0, delta=1e-6)

        temp.SetKelvin(10)
        self.assertAlmostEqual(temp.Kelvin(), 10.0, delta=1e-6)

        temp.SetCelsius(20)
        self.assertAlmostEqual(temp(), 293.15, delta=1e-6)

        temp.SetFahrenheit(30)
        self.assertAlmostEqual(temp.Fahrenheit(), 30.0, delta=1e-6)
        self.assertAlmostEqual(temp(), 272.0388889, delta=1e-6)

    def test_temperatur_operators(self):
        temp = Temperature(20)
        self.assertAlmostEqual(temp(), 20, delta=1e-6)

        temp = Temperature(30)

        self.assertAlmostEqual(temp(), 30, delta=1e-6)

        temp2 = Temperature(temp)
        self.assertTrue(temp == temp2)

        self.assertAlmostEqual((temp + temp2).Kelvin(), 60, delta=1e-6)
        self.assertAlmostEqual((temp + 40).Kelvin(), 70, delta=1e-6)

        self.assertAlmostEqual((temp - temp2).Kelvin(), 0, delta=1e-6)
        self.assertAlmostEqual((temp - 20).Kelvin(), 10.0, delta=1e-6)

        self.assertAlmostEqual((temp * temp2).Kelvin(), 900, delta=1e-6)
        self.assertAlmostEqual((temp * 2).Kelvin(), 60.0, delta=1e-6)

        self.assertAlmostEqual((temp / temp2).Kelvin(), 1.0, delta=1e-6)
        self.assertAlmostEqual((temp / 2).Kelvin(), 15.0, delta=1e-6)

        temp += temp2
        self.assertAlmostEqual(temp.Kelvin(), 60.0, delta=1e-6)
        temp -= temp2
        self.assertAlmostEqual(temp.Kelvin(), 30.0, delta=1e-6)

        temp += 5.0
        self.assertAlmostEqual(temp.Kelvin(), 35.0, delta=1e-6)
        temp -= 5.0
        self.assertAlmostEqual(temp.Kelvin(), 30.0, delta=1e-6)

        temp *= temp2
        self.assertAlmostEqual(temp.Kelvin(), 900, delta=1e-6)
        temp /= temp2
        self.assertAlmostEqual(temp.Kelvin(), 30, delta=1e-6)

        temp *= 4.0
        self.assertAlmostEqual(temp.Kelvin(), 120, delta=1e-6)
        temp /= 4.0
        self.assertAlmostEqual(temp.Kelvin(), 30, delta=1e-6)

        temp3 = Temperature(temp)
        self.assertTrue(temp3 == temp)
        self.assertTrue(temp3 == temp2)

    def test_serialization(self):
        temp = Temperature(55.45)
        self.assertEqual(str(temp), "55.45")


if __name__ == '__main__':
    unittest.main()
