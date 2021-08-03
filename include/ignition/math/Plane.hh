/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_PLANE_HH_
#define IGNITION_MATH_PLANE_HH_

#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>
#include <ignition/math/Plane.hh>
#include <ignition/math/Line2.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Plane Plane.hh ignition/math/Plane.hh
    /// \brief A plane and related functions.
    template<typename T>
    class Plane
    {
      /// \brief Enum used to indicate a side of the plane, no side, or both
      /// sides for entities on the plane.
      /// \sa Side
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

      /// \brief Constructor
      public: Plane()
      : d(0.0)
      {
      }

      /// \brief Constructor from a normal and a distance
      /// \param[in] _normal The plane normal
      /// \param[in] _offset Offset along the normal
      public: Plane(const Vector3<T> &_normal, T _offset = 0.0)
      : normal(_normal), d(_offset)
      {
      }

      /// \brief Constructor
      /// \param[in] _normal The plane normal
      /// \param[in] _size Size of the plane
      /// \param[in] _offset Offset along the normal
      public: Plane(const Vector3<T> &_normal, const Vector2<T> &_size,
                    T _offset)
      {
        this->Set(_normal, _size, _offset);
      }

      /// \brief Copy constructor
      /// \param[in] _plane Plane to copy
      public: Plane(const Plane &_plane)
      : normal(_plane.normal), size(_plane.size), d(_plane.d)
      {}

      /// \brief Destructor
      public: virtual ~Plane() {}

      /// \brief Set the plane
      /// \param[in] _normal The plane normal
      /// \param[in] _offset Offset along the normal
      public: void Set(const Vector3<T> &_normal, T _offset)
      {
        this->normal = _normal;
        this->d = _offset;
      }

      /// \brief Set the plane
      /// \param[in] _normal The plane normal
      /// \param[in] _size Size of the plane
      /// \param[in] _offset Offset along the normal
      public: void Set(const Vector3<T> &_normal, const Vector2<T> &_size,
                       T _offset)
      {
        this->normal = _normal;
        this->size = _size;
        this->d = _offset;
      }

      /// \brief The distance to the plane from the given point. The
      /// distance can be negative, which indicates the point is on the
      /// negative side of the plane.
      /// \param[in] _point 3D point to calculate distance from.
      /// \return Distance from the point to the plane.
      /// \sa Side
      public: T Distance(const Vector3<T> &_point) const
      {
        return this->normal.Dot(_point) - this->d;
      }

      /// \brief Given x,y point find corresponding Z point on plane.
      /// \param[in] x - 2d x coordinate.
      /// \param[in] y - 2d y coordinate.
      /// \return coincident point.
      public: Vector3<T> GetPointOnPlane(const T x, const T y) const
      {
        auto z_val = (this->Normal().Z() != 0) ?
         (this->Offset() - (this->Normal().Dot({x,y,0})))/this->Normal().Z()
         : 0;
        auto coincidentPoint = Vector3<T>{x,y,z_val};
        return coincidentPoint;
      }

      /// \brief Get the intersection of a line with the plane
      /// given the line's gradient and a point in parametrized space.
      /// \param[in] _point A point that lies on a line.
      /// \param[in] _gradient The gradient of the line.
      /// \return The point of intersection. std::nullopt if the line is
      /// parrallel to the plane.
      public: std::optional<Vector3<T>> Intersect(
        const Vector3<T> &_point,
        const Vector3<T> &_gradient) const
      {
        if(abs(this->Normal().Dot(_gradient)) < 1e-6)
        {
          return std::nullopt;
        }
        auto constant = this->Offset() - this->Normal().Dot(_point);
        auto param = constant/this->Normal().Dot(_gradient);
        auto intersection = _point + _gradient*param;
        return intersection;
      }

      /// \brief Get the volume under a plane.
      /// \requires The plane is not parallel to the z axis. And the slopes are
      /// not vertical.
      /// \param[in] _line1 A line that lies on the plane and bounds the x-axis
      /// \param[in] _line2 A line that lies on the plane and bounds the x-axis
      /// \param[in] _ylowerlimit The lower limit of the y-axis
      /// \param[in] _yupperlimit The upper limit of the y-axis
      /// \return The volume under the plane.
      public: T Volume(const Line2<T> &_line1,
                      const Line2<T> &_line2,
                      const T _ylowerlimit,
                      const T _yupperlimit) const
      {
        auto l = _ylowerlimit;
        auto m = _yupperlimit;
        auto k = this->Offset();
        auto j = this->Normal().X()/this->Normal().Z();
        auto n = this->Normal().Y()/this->Normal().Z();

        auto a_1 = _line1.Slope();
        auto a_2 = _line2.Slope();
        auto b_1 = _line1[0].Y() - _line1[0].X() * a_1;
        auto b_2 = _line2[0].Y() - _line2[0].X() * a_1;

        // computed using wolfram alpha:
        // https://www.wolframalpha.com/input/?i=integral+from+m+to+l+of+%28integral+++%28k+-+jx-+ny%29++dy+from+a_1x+%2B+b_1+to+a_2x+%2Bb_2%29+dx
        auto vol = 1/6 *
        (-3 * (l*l - m*m) * (a_1 * (k - b_1 * n)
        + a_2 * (b_2 * n - k) + (b_2 - b_1) * j)
        + (a_1 - a_2) * (l*l*l - m*m*m) * (a_1 * n + a_2 * n + 2 * j)
        + 3 * (b_1 - b_2) * (l - m) * (b_1 * n + b_2 * n - 2 * k));
        return vol;
      }

      /// \brief The side of the plane a point is on.
      /// \param[in] _point The 3D point to check.
      /// \return Plane::NEGATIVE_SIDE if the distance from the point to the
      /// plane is negative, Plane::POSITIVE_SIDE if the distance from the
      /// point to the plane is positive, or Plane::NO_SIDE if the
      /// point is on the plane.
      public: PlaneSide Side(const Vector3<T> &_point) const
      {
        T dist = this->Distance(_point);

        if (dist < 0.0)
          return NEGATIVE_SIDE;

        if (dist > 0.0)
          return POSITIVE_SIDE;

        return NO_SIDE;
      }

      /// \brief The side of the plane a box is on.
      /// \param[in] _box The 3D box to check.
      /// \return Plane::NEGATIVE_SIDE if the distance from the box to the
      /// plane is negative, Plane::POSITIVE_SIDE if the distance from the
      /// box to the plane is positive, or Plane::BOTH_SIDE if the
      /// box is on the plane.
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

      /// \brief Get distance to the plane give an origin and direction
      /// \param[in] _origin the origin
      /// \param[in] _dir a direction
      /// \return the shortest distance
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

      /// \brief Get the plane size
      public: inline const Vector2<T> &Size() const
      {
        return this->size;
      }

      /// \brief Get the plane size
      public: inline Vector2<T> &Size()
      {
        return this->size;
      }

      /// \brief Get the plane offset
      public: inline const Vector3<T> &Normal() const
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline Vector3<T> &Normal()
      {
        return this->normal;
      }

      /// \brief Get the plane offset
      public: inline T Offset() const
      {
        return this->d;
      }

      /// \brief Equal operator
      /// \param _p another plane
      /// \return itself
      public: Plane<T> &operator=(const Plane<T> &_p)
      {
        this->normal = _p.normal;
        this->size = _p.size;
        this->d = _p.d;

        return *this;
      }

      /// \brief Plane normal
      private: Vector3<T> normal;

      /// \brief Plane size
      private: Vector2<T> size;

      /// \brief Plane offset
      private: T d;
    };

    typedef Plane<int> Planei;
    typedef Plane<double> Planed;
    typedef Plane<float> Planef;
    }
  }
}

#endif
