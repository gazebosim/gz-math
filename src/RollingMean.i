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

%module rollingMean
%{
#include <ignition/math/RollingMean.hh>
%}

namespace ignition
{
  namespace math
  {
    class RollingMean
    {

      public: explicit RollingMean(size_t _windowSize = 10);
      public: ~RollingMean();
      public: double Mean() const;
      public: size_t Count() const;
      public: void Push(double _value);
      public: void Clear();
      public: void SetWindowSize(size_t _windowSize);
      public: size_t WindowSize() const;

      #ifdef _WIN32
      #pragma warning(push)
      #pragma warning(disable: 4251)
      #endif
          private: std::unique_ptr<RollingMeanPrivate> dataPtr;
      #ifdef _WIN32
      #pragma warning(pop)
      #endif
    };
  }
}