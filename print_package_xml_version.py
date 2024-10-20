# Copyright (C) 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
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

import re
import sys
from xml.etree import ElementTree as ET

if len(sys.argv) != 2:
    print('need to specify the path to a package.xml', file=sys.stderr)
    exit()

file_name = sys.argv[1]

doc = ET.parse(file_name)
root = doc.getroot()
if root.tag != 'package':
    print('Invalid package.xml file', file=sys.stderr)
    exit()

version_str = root.find('version').text

# validate version string using regex from catkin_pkg
# https://github.com/ros-infrastructure/catkin_pkg/blob/1.0.0/src/catkin_pkg/package_version.py#L55-L58
match = re.match(r'^(\d+)\.(\d+)\.(\d+)$', version_str)
if match is None:
    raise ValueError('Invalid version string, must be int.int.int: "%s"' % version)

print(version_str, end='')
