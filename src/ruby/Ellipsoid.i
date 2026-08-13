/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

%module ellipsoid
%{
#include "gz/math/Ellipsoid.hh"
#include "gz/math/MassMatrix3.hh"
#include "gz/math/Material.hh"
#include "gz/math/Plane.hh"
%}

%include "typemaps.i"
%typemap(out) (std::optional< gz::math::Vector3< double > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new gz::math::Vector3< double >(static_cast< const gz::math::Vector3< double >& >((*(&result)).value()))),
      SWIGTYPE_p_gz__math__Vector3T_double_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}

namespace gz
{
  namespace math
  {
    template<typename Precision>
    class Ellipsoid
    {
      public: Ellipsoid() = default;

      public: explicit Ellipsoid(const gz::math::Vector3<Precision> &_radii);

      public: Ellipsoid(const gz::math::Vector3<Precision> &_radii,
                  const gz::math::Material &_mat);

      public: ~Ellipsoid() = default;

      public: gz::math::Vector3<Precision> Radii() const;

      public: void SetRadii(const gz::math::Vector3<Precision> &_radii);

      public: const gz::math::Material &Mat() const;

      public: void SetMat(const gz::math::Material &_mat);

      public: bool operator==(const Ellipsoid &_ellipsoid) const;

      public: Precision Volume() const;

      public: Precision VolumeBelow(const gz::math::Plane<Precision> &_plane) const;

      public: std::optional<gz::math::Vector3<Precision>>
        CenterOfVolumeBelow(const gz::math::Plane<Precision> &_plane) const;

      public: Precision DensityFromMass(const Precision _mass) const;

      public: bool SetDensityFromMass(const Precision _mass);
    };
    %template(Ellipsoidd) Ellipsoid<double>;
  }
}
