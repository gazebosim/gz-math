# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This example will only work if the Python interface library was compiled and
# installed.
#
# Modify the PYTHONPATH environment variable to include the ignition math
# library install path. For example, if you install to /user:
#
# $ export PYTHONPATH=/usr/lib/python:$PYTHONPATH
#
import ignition.math

va = ignition.math.Vector2d(1, 2)
vb = ignition.math.Vector2d(3, 4)
vc = ignition.math.Vector2d(vb)

print("va = {} {}\n".format(va.X(), va.Y()))
print("vb = {} {}\n".format(vb.X(), vb.Y()))
print("vc = {} {}\n".format(vc.X(), vc.Y()))

vb += va
print("vb += va: {} {}\n".format(vb.X(), vb.Y()))

vb.Normalize()
print("vb.Normalize = {} {}\n".format(vb.X(), vb.Y()))

print("vb.Distance(va) = {}\n".format(vb.Distance(va)))