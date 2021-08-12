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
import math
from ignition.math import Rand
from ignition.math import SignalMaximum
from ignition.math import SignalMinimum
from ignition.math import SignalMean
from ignition.math import SignalVariance
from ignition.math import SignalRootMeanSquare
from ignition.math import SignalMaxAbsoluteValue
from ignition.math import SignalStats
from ignition.math import SignalStatistic


class TestSignalStats(unittest.TestCase):

    def test_signal_maximum_constructor(self):
        # Constructor
        max = SignalMaximum()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)
        self.assertEqual(max.ShortName(), "max")

        # Reset
        max.Reset()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

    def test_signal_maximum_constant_values(self):
        # Constant values, max should match
        max = SignalMaximum()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                max.InsertData(value)
                self.assertAlmostEqual(max.Value(), value)
                self.assertEqual(max.Count(), i)

            # Reset
            max.Reset()
            self.assertAlmostEqual(max.Value(), 0.0)
            self.assertEqual(max.Count(), 0)

    def test_signal_maximum_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should always match positive value
        max = SignalMaximum()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):

            for i in range(1, 11):
                max.InsertData(value * i)
                self.assertAlmostEqual(max.Value(), value * i)
                max.InsertData(-value * i)
                self.assertAlmostEqual(max.Value(), value * i)
                self.assertEqual(max.Count(), i*2)

            # Reset
            max.Reset()
            self.assertAlmostEqual(max.Value(), 0.0)
            self.assertEqual(max.Count(), 0)

    def test_signal_mean_constructor(self):
        # Constructor
        mean = SignalMean()
        self.assertAlmostEqual(mean.Value(), 0.0)
        self.assertEqual(mean.Count(), 0)
        self.assertEqual(mean.ShortName(), "mean")

        # Reset
        mean.Reset()
        self.assertAlmostEqual(mean.Value(), 0.0)
        self.assertEqual(mean.Count(), 0)

    def test_signal_mean_constant_values(self):
        # Constant values, mean should match
        mean = SignalMean()
        self.assertAlmostEqual(mean.Value(), 0.0)
        self.assertEqual(mean.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                mean.InsertData(value)
                self.assertAlmostEqual(mean.Value(), value)
                self.assertEqual(mean.Count(), i)

            # Reset
            mean.Reset()
            self.assertAlmostEqual(mean.Value(), 0.0)
            self.assertEqual(mean.Count(), 0)

    def test_signal_mean_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should be zero every other time
        mean = SignalMean()
        self.assertAlmostEqual(mean.Value(), 0.0)
        self.assertEqual(mean.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                mean.InsertData(value * i)
                mean.InsertData(-value * i)
                self.assertAlmostEqual(mean.Value(), 0.0)
                self.assertEqual(mean.Count(), i*2)

            # Reset
            mean.Reset()
            self.assertAlmostEqual(mean.Value(), 0.0)
            self.assertEqual(mean.Count(), 0)

    def test_signal_minimum_constructor(self):
        # Constructor
        min = SignalMinimum()
        self.assertAlmostEqual(min.Value(), 0.0)
        self.assertEqual(min.Count(), 0)
        self.assertEqual(min.ShortName(), "min")

        # Reset
        min.Reset()
        self.assertAlmostEqual(min.Value(), 0.0)
        self.assertEqual(min.Count(), 0)

    def test_signal_minimum_constant_values(self):
        # Constant values, min should match
        min = SignalMinimum()
        self.assertAlmostEqual(min.Value(), 0.0)
        self.assertEqual(min.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                min.InsertData(value)
                self.assertAlmostEqual(min.Value(), value)
                self.assertEqual(min.Count(), i)

            # Reset
            min.Reset()
            self.assertAlmostEqual(min.Value(), 0.0)
            self.assertEqual(min.Count(), 0)

    def test_signal_minimum_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # Should always match negative value
        min = SignalMinimum()
        self.assertAlmostEqual(min.Value(), 0.0)
        self.assertEqual(min.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                min.InsertData(value * i)
                min.InsertData(-value * i)
                self.assertAlmostEqual(min.Value(), -value * i)
                self.assertEqual(min.Count(), i*2)

            # Reset
            min.Reset()
            self.assertAlmostEqual(min.Value(), 0.0)
            self.assertEqual(min.Count(), 0)

    def test_signal_root_mean_square(self):
        # Constructor
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.Value(), 0.0)
        self.assertEqual(rms.Count(), 0)
        self.assertEqual(rms.ShortName(), "rms")

        # Reset
        rms.Reset()
        self.assertAlmostEqual(rms.Value(), 0.0)
        self.assertEqual(rms.Count(), 0)

    def test_signal_root_mean_square_constant_values(self):
        # Constant values, rms should match
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.Value(), 0.0)
        self.assertEqual(rms.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                rms.InsertData(value)
                self.assertAlmostEqual(rms.Value(), value)
                self.assertEqual(rms.Count(), i)

            # Reset
            rms.Reset()
            self.assertAlmostEqual(rms.Value(), 0.0)
            self.assertEqual(rms.Count(), 0)

    def test_signal_root_mean_square_alternating_values(self):
        # Values with alternating sign, same magnitude
        # rms should match absolute value every time
        rms = SignalRootMeanSquare()
        self.assertAlmostEqual(rms.Value(), 0.0)
        self.assertEqual(rms.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                rms.InsertData(value)
                self.assertAlmostEqual(rms.Value(), value)
                self.assertEqual(rms.Count(), i*2-1)

                rms.InsertData(-value)
                self.assertAlmostEqual(rms.Value(), value)
                self.assertEqual(rms.Count(), i*2)

            # Reset
            rms.Reset()
            self.assertAlmostEqual(rms.Value(), 0.0)
            self.assertEqual(rms.Count(), 0)

    def test_signal_max_absolute_value_constructor(self):
        # Constructor
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)
        self.assertEqual(max.ShortName(), "maxAbs")

        # Reset
        max.Reset()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

    def test_signal_max_absolute_value_constant_values(self):
        # Constant values, max should match
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                max.InsertData(value)
                self.assertAlmostEqual(max.Value(), value)
                self.assertEqual(max.Count(), i)

            # Reset
            max.Reset()
            self.assertAlmostEqual(max.Value(), 0.0)
            self.assertEqual(max.Count(), 0)

    def test_signal_max_absolute_value_alternating_values(self):
        # Values with alternating sign, increasing magnitude
        # max should match absolute value every time
        max = SignalMaxAbsoluteValue()
        self.assertAlmostEqual(max.Value(), 0.0)
        self.assertEqual(max.Count(), 0)

        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                max.InsertData(value * i)
                self.assertAlmostEqual(max.Value(), value * i)
                self.assertEqual(max.Count(), i*2-1)

                max.InsertData(-value * i)
                self.assertAlmostEqual(max.Value(), value * i)
                self.assertEqual(max.Count(), i*2)

            # Reset
            max.Reset()
            self.assertAlmostEqual(max.Value(), 0.0)
            self.assertEqual(max.Count(), 0)

    def test_signal_variance_constructor(self):
        var = SignalVariance()
        self.assertAlmostEqual(var.Value(), 0.0)
        self.assertEqual(var.Count(), 0)
        self.assertEqual(var.ShortName(), "var")

        # Reset
        var.Reset()
        self.assertAlmostEqual(var.Value(), 0.0)
        self.assertEqual(var.Count(), 0)

    def test_signal_variance_one_value(self):
        # Add one value, expect 0.0 variance
        values = {0, 1.0, 10.0, -100.0}
        for value in values:
            var = SignalVariance()
            var.InsertData(value)
            self.assertEqual(var.Count(), 1)
            self.assertAlmostEqual(0.0, var.Value())

            # Reset
            var.Reset()
            self.assertAlmostEqual(0.0, var.Value())
            self.assertEqual(var.Count(), 0)

    def test_signal_variance_constant_values(self):
        # Constant values, expect 0.0 variance
        var = SignalVariance()
        value = 3.14159

        # Loop two times to verify Reset
        for j in range(2):
            for i in range(1, 11):
                var.InsertData(value)
                self.assertAlmostEqual(0.0, var.Value())
                self.assertEqual(var.Count(), i)

            # Reset
            var.Reset()
            self.assertAlmostEqual(var.Value(), 0.0)
            self.assertEqual(var.Count(), 0)

    def test_signal_variance_random_values(self):
        # Random normally distributed values
        # The sample variance has the following variance:
        # 2 variance^2 / (count - 1)
        # en.wikipedia.org/wiki/Variance#Distribution_of_the_sample_variance
        # We will use 5 sigma (4e-5 chance of failure)
        var = SignalVariance()
        stdDev = 3.14159
        count = 10000
        sigma = 5.0
        for i in range(count):
            var.InsertData(Rand.DblNormal(0.0, stdDev))

        variance = stdDev*stdDev
        sampleVariance2 = 2 * variance*variance / (count - 1)
        self.assertAlmostEqual(var.Value(), variance,
                               delta=sigma*math.sqrt(sampleVariance2))

        # Reset
        var.Reset()
        self.assertAlmostEqual(var.Value(), 0.0)
        self.assertEqual(var.Count(), 0)

    def test_signal_stats_constructor(self):
        # Constructor
        stats = SignalStats()
        self.assertTrue(stats.Map().empty())
        self.assertEqual(stats.Count(), 0)

        stats2 = SignalStats(stats)
        self.assertEqual(stats.Count(), stats2.Count())

        # Reset
        stats.Reset()
        self.assertTrue(stats.Map().empty())
        self.assertEqual(stats.Count(), 0)

    def test_01_signal_stats_intern_statistic(self):
        # InsertStatistic
        stats = SignalStats()
        self.assertTrue(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("max"))
        self.assertFalse(stats.InsertStatistic("max"))
        self.assertFalse(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("maxAbs"))
        self.assertFalse(stats.InsertStatistic("maxAbs"))
        self.assertFalse(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("mean"))
        self.assertFalse(stats.InsertStatistic("mean"))
        self.assertFalse(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("min"))
        self.assertFalse(stats.InsertStatistic("min"))
        self.assertFalse(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("rms"))
        self.assertFalse(stats.InsertStatistic("rms"))
        self.assertFalse(stats.Map().empty())

        self.assertTrue(stats.InsertStatistic("var"))
        self.assertFalse(stats.InsertStatistic("var"))
        self.assertFalse(stats.Map().empty())

        self.assertFalse(stats.InsertStatistic("FakeStatistic"))

        # Map with no data
        map = stats.Map()
        self.assertFalse(map.empty())
        self.assertEqual(map.size(), 6)
        self.assertEqual(map.count("max"), 1)
        self.assertEqual(map.count("maxAbs"), 1)
        self.assertEqual(map.count("mean"), 1)
        self.assertEqual(map.count("min"), 1)
        self.assertEqual(map.count("rms"), 1)
        self.assertEqual(map.count("var"), 1)
        self.assertEqual(map.count("FakeStatistic"), 0)

        stats2 = SignalStats(stats)
        map2 = stats2.Map()
        self.assertFalse(map2.empty())
        self.assertEqual(map.size(), map2.size())
        self.assertEqual(map.count("max"), map2.count("max"))
        self.assertEqual(map.count("maxAbs"), map2.count("maxAbs"))
        self.assertEqual(map.count("mean"), map2.count("mean"))
        self.assertEqual(map.count("min"), map2.count("min"))
        self.assertEqual(map.count("rms"), map2.count("rms"))
        self.assertEqual(map.count("var"), map2.count("var"))
        self.assertEqual(map.count("FakeStatistic"),
                         map2.count("FakeStatistic"))

    def test_02_signal_stats_intern_statistic(self):
        # InsertStatistics
        stats = SignalStats()
        self.assertFalse(stats.InsertStatistics(""))
        self.assertTrue(stats.Map().empty())

        self.assertTrue(stats.InsertStatistics("maxAbs,rms"))
        self.assertEqual(stats.Map().size(), 2)
        self.assertFalse(stats.InsertStatistics("maxAbs,rms"))
        self.assertFalse(stats.InsertStatistics("maxAbs"))
        self.assertFalse(stats.InsertStatistics("rms"))
        self.assertEqual(stats.Map().size(), 2)

        self.assertFalse(stats.InsertStatistics("mean,FakeStatistic"))
        self.assertEqual(stats.Map().size(), 3)

        self.assertFalse(stats.InsertStatistics("var,FakeStatistic"))
        self.assertEqual(stats.Map().size(), 4)

        self.assertFalse(stats.InsertStatistics("max,FakeStatistic"))
        self.assertEqual(stats.Map().size(), 5)

        self.assertFalse(stats.InsertStatistics("min,FakeStatistic"))
        self.assertEqual(stats.Map().size(), 6)

        self.assertFalse(stats.InsertStatistics("FakeStatistic"))
        self.assertEqual(stats.Map().size(), 6)

        # Map with no data
        map = stats.Map()
        self.assertFalse(map.empty())
        self.assertEqual(map.size(), 6)
        self.assertEqual(map.count("max"), 1)
        self.assertEqual(map.count("maxAbs"), 1)
        self.assertEqual(map.count("mean"), 1)
        self.assertEqual(map.count("min"), 1)
        self.assertEqual(map.count("rms"), 1)
        self.assertEqual(map.count("var"), 1)
        self.assertEqual(map.count("FakeStatistic"), 0)

    def test_signal_stats_alternating_values(self):
        # Add some statistics
        stats = SignalStats()
        self.assertTrue(stats.InsertStatistics("max,maxAbs,mean,min,rms"))
        self.assertEqual(stats.Map().size(), 5)

        # No data yet
        self.assertEqual(stats.Count(), 0)

        # Insert data with alternating signs
        value = 3.14159
        stats.InsertData(value)
        stats.InsertData(-value)
        self.assertEqual(stats.Count(), 2)

        map = stats.Map()
        self.assertAlmostEqual(map["max"], value)
        self.assertAlmostEqual(map["maxAbs"], value)
        self.assertAlmostEqual(map["min"], -value)
        self.assertAlmostEqual(map["rms"], value)
        self.assertAlmostEqual(map["mean"], 0.0)

        # test operator=
        copy = SignalStats(stats)
        self.assertEqual(copy.Count(), 2)
        map = stats.Map()
        self.assertEqual(map.size(), 5)
        self.assertAlmostEqual(map["max"], value)
        self.assertAlmostEqual(map["maxAbs"], value)
        self.assertAlmostEqual(map["min"], -value)
        self.assertAlmostEqual(map["rms"], value)
        self.assertAlmostEqual(map["mean"], 0.0)

        stats.Reset()
        self.assertEqual(stats.Map().size(), 5)
        self.assertEqual(stats.Count(), 0)
        map = stats.Map()
        self.assertAlmostEqual(map["max"], 0.0)
        self.assertAlmostEqual(map["maxAbs"], 0.0)
        self.assertAlmostEqual(map["min"], 0.0)
        self.assertAlmostEqual(map["rms"], 0.0)
        self.assertAlmostEqual(map["mean"], 0.0)


if __name__ == '__main__':
    unittest.main()
