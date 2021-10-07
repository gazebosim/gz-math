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

import unittest

import ignition
from ignition.math import Vertexi


class TestVertex(unittest.TestCase):

    def test_accessors(self):
        data = 5
        vertex = Vertexi('', data)
        print(dir(vertex.data()))
        self.assertEqual(vertex.name(), '')
        vertex.set_name('new_name')
        self.assertEqual('new_name', vertex.name())
        self.assertEqual(vertex.data(), data)
        self.assertEqual(vertex.id(), ignition.math.KNULLID)
        self.assertFalse(vertex.valid())

        name = 'my_vertex'
        data = 10
        id_var = 2
        vertex = Vertexi(name, data, id)
        print(type(vertex.id()))
        self.assertEqual(vertex.name(), name)
        self.assertEqual(vertex.id(), id_var)
        self.assertEqual(vertex.data(), data)
        #
        # # Modify the data
        # # vertex.data() += 1
        # # self.assertEqual(vertex.data(), data + 1)
        # self.assertTrue(vertex.valid())
#
# /////////////////////////////////////////////////
# TEST(VertexTest, StreamInsertion)
# {
#   std::string name = "my_vertex"
#   data = 10
#   VertexId id = 2
#   Vertex<int> vertex(name, data, id)
#
#   std::ostringstream output
#   output << vertex
#
#   std::string expectedOutput = "  " + std::to_string(id) + " [label=\"" + name +
#     " (" + std::to_string(id) + ")\"]\n"
#   self.assertEqual(output.str(), expectedOutput)
# }



if __name__ == '__main__':
    unittest.main()
