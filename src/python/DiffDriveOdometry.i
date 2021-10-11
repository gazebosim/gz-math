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

%module diffdriveodometry
%{
#include <chrono>
#include <memory>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Export.hh>
#include <ignition/math/config.hh>
%}

%include "typemaps.i"
%typemap(in, numinputs=1) const std::chrono::steady_clock::time_point &_time %{
long val2;
if (!SWIG_IsOK(SWIG_AsVal_long(swig_obj[1], &val2))) {
  SWIG_exception_fail(SWIG_ArgError(SWIG_AsVal_long(swig_obj[1], &val2)), "in method '" "DiffDriveOdometry_init" "', argument " "2"" of type '" "long""'");
}

std::chrono::milliseconds dur;
dur = std::chrono::milliseconds(val2);

using std::chrono::duration_cast;
arg2 = new std::chrono::steady_clock::time_point();
*arg2 += duration_cast<std::chrono::steady_clock::duration>(dur);
%}

%typemap(in, numinputs=1) const std::chrono::steady_clock::time_point &_timePoint %{
long val4;
if (!SWIG_IsOK(SWIG_AsVal_long(swig_obj[3], &val4))) {
  SWIG_exception_fail(SWIG_ArgError(SWIG_AsVal_long(swig_obj[3], &val4)), "in method '" "DiffDriveOdometry_update" "', argument " "2"" of type '" "long""'");
}
std::chrono::milliseconds dur;
dur = std::chrono::milliseconds(val4);

using std::chrono::duration_cast;
arg4 = new std::chrono::steady_clock::time_point();
*arg4 += duration_cast<std::chrono::steady_clock::duration>(dur);
%}

namespace ignition
{
  namespace math
  {
    class DiffDriveOdometry
    {
      %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

      public: explicit DiffDriveOdometry(size_t _windowSize = 10);

      public: ~DiffDriveOdometry();

      public: void Init(const std::chrono::steady_clock::time_point &_time);

      public: bool Initialized() const;

      public: bool Update(const Angle &_leftPos, const Angle &_rightPos,
                          const std::chrono::steady_clock::time_point &_timePoint);

      public: const Angle &Heading() const;

      public: double X() const;

      public: double Y() const;

      public: double LinearVelocity() const;

      public: const Angle &AngularVelocity() const;

      public: void SetWheelParams(double _wheelSeparation,
                      double _leftWheelRadius, double _rightWheelRadius);

      public: void SetVelocityRollingWindowSize(size_t _size);
    };
  }
}
