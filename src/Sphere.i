/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifdef SWIGRUBY
%begin %{
#define HAVE_ISFINITE 1
%}
#endif

%module angle
%{
#include <ignition/math/Sphere.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename Precision>
    class Sphere
    {
      public: Sphere();
      public: Sphere(const Sphere &_sphere);
      public: Sphere(const Sphere &&_sphere);
      public: explicit Sphere(const Precision _radius);
      public: Sphere(const Precision _radius, const Material &_mat);
      public: ~Sphere() = default;
      public: Precision Radius() const;
      public: void SetRadius(const Precision _radius);
      public: const ignition::math::Material &Material() const;
      public: void SetMaterial(const ignition::math::Material &_mat);
      public: bool MassMatrix(MassMatrix3d &_massMat) const;
      public: bool operator==(const Sphere &_sphere) const;
      public: bool operator!=(const Sphere &_sphere) const;
      public: Sphere &operator=(const Sphere &_sphere);
      public: Sphere &operator=(Sphere &&_sphere);
      public: Precision Volume() const;
      public: Precision DensityFromMass(const Precision _mass) const;
      public: bool SetDensityFromMass(const Precision _mass);
    };

    %template(Spherei) Sphere<int>;
    %template(Sphered) Sphere<double>;
    %template(Spheref) Sphere<float>;
  }
}
