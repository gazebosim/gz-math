/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

%module vertex
%{
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include <ignition/math/graph/Vertex.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Helpers.hh>
%}

%include "stdint.i"
%include "typemaps.i"
%typemap(in) const uint64_t _id %{
long val2;
if (!SWIG_IsOK(SWIG_AsVal_long(swig_obj[2], &val2))) {
  SWIG_exception_fail(SWIG_ArgError(SWIG_AsVal_long(swig_obj[2], &val2)), "in method '" "Vertex_new" "', argument " "2"" of type '" "long""'");
}
arg2 = static_cast<uint64_t>(val2);
%}

namespace ignition
{
  namespace math
  {
    namespace graph
    {
      using VertexId_P = std::pair<uint64_t, uint64_t>;

      static const uint64_t kNullId = MAX_UI64;

      template<typename V>
      class Vertex
      {
        %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

        public: Vertex(const std::string &_name,
                       const V &_data = V(),
                       const uint64_t _id = kNullId);
        public: const V Data() const;
        public: V Data();
        public: void SetData(const V &_data);
        public: uint64_t Id() const;
        public: void SetId(const uint64_t _id) const;
        public: const std::string &Name() const;
        public: void SetName(const std::string &_name);

        public: bool Valid() const;
      };
      %template(Vertexi) Vertex<int>;
    }
  }
}
