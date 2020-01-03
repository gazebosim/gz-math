# Copyright (C) 2020 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env ruby

require 'test/unit/ui/console/testrunner'
require 'test/unit'
require 'math'

class Sphere_TEST < Test::Unit::TestCase
  def test_functions
    sphere = Ignition::Math::Sphered.new
    assert(sphere.Radius().zero?, "Default radius should be zero")

    sphere.SetRadius(1.23)
    assert(sphere.Radius() == 1.23, "Radius should equal 1.23")
 
    sphere.SetRadius(2.0)
    assert(sphere.Radius() == 2.0, "Radius should equal 2.0")
    assert(sphere.Volume() == (4.0/3.0) * Math::PI * 8,
           "Volume should equal 33.51")

    sphere2 = Ignition::Math::Sphered.new
    assert(sphere2 != sphere, "Sphere2 should not equal sphere")
    sphere2 = sphere
    assert(sphere2 == sphere, "Sphere2 should equal sphere")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Sphere_TEST).passed? ? 0 : -1
