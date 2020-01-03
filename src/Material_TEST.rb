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

class Material_TEST < Test::Unit::TestCase
  def test_construction
    material = Ignition::Math::Material.new
    assert(sphere.Type() == Ignition::Math::MaterialType::UNKNOWN_MATERIAL,
           "Default material should be unknown")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Material_TEST).passed? ? 0 : -1
