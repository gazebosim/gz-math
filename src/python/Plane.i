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

%module plane
%{
#include <ignition/math/Plane.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Line2.hh>
#include <ignition/math/Quaternion.hh>
#include <optional>
%}

%include "typemaps.i"
%typemap(out) (std::optional< ignition::math::Vector3< double > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new ignition::math::Vector3< double >(static_cast< const ignition::math::Vector3< double >& >((*(&result)).value()))),
      SWIGTYPE_p_ignition__math__Vector3T_double_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}


namespace ignition
{
  namespace math
  {
    template<typename T>
    class Plane
    {
      public: enum PlaneSide
      {
        /// \brief Negative side of the plane. This is the side that is
        /// opposite the normal.
        NEGATIVE_SIDE = 0,

        /// \brief Positive side of the plane. This is the side that has the
        /// normal vector.
        POSITIVE_SIDE = 1,

        /// \brief On the plane.
        NO_SIDE = 2,

        /// \brief On both sides of the plane.
        BOTH_SIDE = 3
      };

      public: Plane()
      : d(0.0)
      {
      }

      public: Plane(const Vector3<T> &_normal, T _offset = 0.0)
      : normal(_normal), d(_offset)
      {
      }

      public: Plane(const Vector3<T> &_normal, const Vector2<T> &_size,
                    T _offset)
      {
        this->Set(_normal, _size, _offset);
      }

      public: Plane(const Plane &_plane)
      : normal(_plane.normal), size(_plane.size), d(_plane.d)
      {}

      public: virtual ~Plane() {}

      public: void Set(const Vector3<T> &_normal, T _offset)
      {
        this->normal = _normal;
        this->d = _offset;
      }

      public: void Set(const Vector3<T> &_normal, const Vector2<T> &_size,
                       T _offset)
      {
        this->normal = _normal;
        this->size = _size;
        this->d = _offset;
      }

      public: T Distance(const Vector3<T> &_point) const
      {
        return this->normal.Dot(_point) - this->d;
      }

      public: std::optional<Vector3<T>> Intersection(
        const Vector3<T> &_point,
        const Vector3<T> &_gradient,
        const double &_tolerance = 1e-6) const
      {
        if (std::abs(this->Normal().Dot(_gradient)) < _tolerance)
        {
          return std::nullopt;
        }
        auto constant = this->Offset() - this->Normal().Dot(_point);
        auto param = constant / this->Normal().Dot(_gradient);
        auto intersection = _point + _gradient*param;

        if (this->Size() == Vector2<T>(0, 0))
          return intersection;

        // Check if the point is within the size bounds
        // To do this we create a Quaternion using Angle, Axis constructor and
        // rotate the Y and X axis the same amount as the normal.
        auto dotProduct = Vector3<T>::UnitZ.Dot(this->Normal());
        auto angle = acos(dotProduct / this->Normal().Length());
        auto axis = Vector3<T>::UnitZ.Cross(this->Normal().Normalized());
        Quaternion<T> rotation(axis, angle);

        Vector3<T> rotatedXAxis = rotation * Vector3<T>::UnitX;
        Vector3<T> rotatedYAxis = rotation * Vector3<T>::UnitY;

        auto xBasis = rotatedXAxis.Dot(intersection);
        auto yBasis = rotatedYAxis.Dot(intersection);

        if (std::abs(xBasis) < this->Size().X() / 2 &&
            std::abs(yBasis) < this->Size().Y() / 2)
        {
          return intersection;
        }
        return std::nullopt;
      }

      public: PlaneSide Side(const Vector3<T> &_point) const
      {
        T dist = this->Distance(_point);

        if (dist < 0.0)
          return NEGATIVE_SIDE;

        if (dist > 0.0)
          return POSITIVE_SIDE;

        return NO_SIDE;
      }

      public: PlaneSide Side(const math::AxisAlignedBox &_box) const
      {
        double dist = this->Distance(_box.Center());
        double maxAbsDist = this->normal.AbsDot(_box.Size()/2.0);

        if (dist < -maxAbsDist)
          return NEGATIVE_SIDE;

        if (dist > maxAbsDist)
          return POSITIVE_SIDE;

        return BOTH_SIDE;
      }

      public: T Distance(const Vector3<T> &_origin,
                         const Vector3<T> &_dir) const
      {
        T denom = this->normal.Dot(_dir);

        if (std::abs(denom) < 1e-3)
        {
          // parallel
          return 0;
        }
        else
        {
          T nom = _origin.Dot(this->normal) - this->d;
          T t = -(nom/denom);
          return t;
        }
      }

      public: inline const Vector2<T> &Size() const
      {
        return this->size;
      }

      public: inline Vector2<T> &Size()
      {
        return this->size;
      }

      public: inline const Vector3<T> &Normal() const
      {
        return this->normal;
      }

      public: inline Vector3<T> &Normal()
      {
        return this->normal;
      }

      public: inline T Offset() const
      {
        return this->d;
      }

      private: Vector3<T> normal;

      private: Vector2<T> size;

      private: T d;
    };

    %template(Planed) Plane<double>;
  }
}
