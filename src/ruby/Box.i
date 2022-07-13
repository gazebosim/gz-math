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

%module box
%{
#include <gz/math/Box.hh>
#include <gz/math/config.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Material.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Vector3.hh>

#include "gz/math/detail/WellOrderedVector.hh"

#include <set>
#include <optional>
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

%typemap(out) (std::optional< gz::math::Vector3< int > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new gz::math::Vector3< int >(static_cast< const gz::math::Vector3< int >& >((*(&result)).value()))),
      SWIGTYPE_p_gz__math__Vector3T_int_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}

#include "std_set.i"
%template(SetBoxDouble) std::set<gz::math::Vector3<double>, gz::math::WellOrderedVectors<double>>;
%template(SetBoxInt) std::set<gz::math::Vector3<int>, gz::math::WellOrderedVectors<int>>;

namespace gz
{
  namespace math
  {
    template<typename Precision>
    class Box
    {
      %rename("%(undercase)s", %$isfunction, notregexmatch$name="^[A-Z]*$") "";

      public: Box() = default;

      public: Box(const Precision _length,
                  const Precision _width,
                  const Precision _height);

      public: Box(const Precision _length, const Precision _width,
                  const Precision _height,
                  const gz::math::Material &_mat);

      public: explicit Box(const gz::math::Vector3<Precision> &_size);

      public: Box(const gz::math::Vector3<Precision> &_size,
                  const gz::math::Material &_mat);

      public: virtual ~Box() = default;

      public: gz::math::Vector3<Precision> Size() const;

      public: void SetSize(const gz::math::Vector3<Precision> &_size);

      public: void SetSize(const Precision _length,
                           const Precision _width,
                           const Precision _height);

      public: bool operator==(const Box<Precision> &_b) const;

      public: bool operator!=(const Box<Precision> &_b) const;

      public: const gz::math::Material &Material() const;

      public: void SetMaterial(const gz::math::Material &_mat);

      public: Precision Volume() const;

      public: Precision VolumeBelow(const gz::math::Plane<Precision> &_plane) const;

      public: std::optional<gz::math::Vector3<Precision>>
        CenterOfVolumeBelow(const gz::math::Plane<Precision> &_plane) const;

      public: std::set<gz::math::Vector3<Precision>, gz::math::WellOrderedVectors<Precision>>
        VerticesBelow(const gz::math::Plane<Precision> &_plane) const;

      public: Precision DensityFromMass(const Precision _mass) const;

      public: bool SetDensityFromMass(const Precision _mass);

      public: bool MassMatrix(gz::math::MassMatrix3<Precision> &_massMat) const;

      public: std::set<gz::math::Vector3<Precision>, gz::math::WellOrderedVectors<Precision>> Intersections(
        const gz::math::Plane<Precision> &_plane) const;

      private: gz::math::Vector3<Precision> size = gz::math::Vector3<Precision>::Zero;

      private: gz::math::Material material;
    };

    %template(Boxi) Box<int>;
    %template(Boxd) Box<double>;
  }
}
