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
#ifndef GZ_MATH_QUATERNION_HH_
#define GZ_MATH_QUATERNION_HH_

#include <gz/math/Helpers.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/config.hh>

namespace gz::math
{
  // Inline bracket to help doxygen filtering.
  inline namespace GZ_MATH_VERSION_NAMESPACE {
  //
  template <typename T> class Matrix3;

  /// \class Quaternion Quaternion.hh gz/math/Quaternion.hh
  /// \brief A quaternion class that represents 3D rotations and
  /// orientations. Four scalar values, [w,x,y,z], are used represent
  /// orientations and rotations.
  ///
  /// The following two type definitions are provided:
  ///
  /// * \ref Quaternionf
  /// * \ref Quaterniond
  ///
  /// ## Examples
  ///
  /// * C++
  ///
  /// \snippet examples/quaternion_example.cc complete
  ///
  /// * Ruby
  ///
  /// \code{.rb}
  /// # Modify the RUBYLIB environment variable to include the Gazebo Math
  /// # library install path. For example, if you install to /user:
  /// #
  /// # $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
  /// #
  /// require 'gz/math'
  ///
  /// q = Gz::Math::Quaterniond.new
  /// printf("A default quaternion has the following values\n"+
  ///        "\tW=%f X=%f Y=%f Z=%f\n", q.W(), q.X(), q.Y(), q.Z())
  ///
  /// q = Gz::Math::Quaterniond.Identity
  /// printf("The identity quaternion has the following values\n" +
  ///        "\tW=%f X=%f Y=%f Z=%f\n", q.W(), q.X(), q.Y(), q.Z())
  ///
  /// q2 = Gz::Math::Quaterniond.new(0, 0, 3.14)
  /// printf("A quaternion initialized from roll=0, pitch=0, and yaw=3.14 " +
  ///        "has the following values\n" +
  ///        "\tW=%f X=%f Y=%f Z=%f\n", q2.W(), q2.X(), q2.Y(), q2.Z())
  ///
  /// euler = q2.Euler()
  /// printf("Getting back the euler angles from the quaternion\n" +
  ///        "\troll=%f pitch=%f yaw=%f\n", euler.X(), euler.Y(), euler.Z())
  ///
  /// \endcode
  template<typename T>
  class Quaternion
  {
    /// \brief A Quaternion initialized to identity.
    /// This is equivalent to math::Quaternion<T>(1, 0, 0, 0)
    public: static const Quaternion &Identity;

    /// \brief A Quaternion initialized to zero.
    /// This is equivalent to math::Quaternion<T>(0, 0, 0, 0)
    public: static const Quaternion &Zero;

    /// \brief Default Constructor
    public: constexpr Quaternion()
    : qw(1), qx(0), qy(0), qz(0)
    {
      // quaternion not normalized, because that breaks
      // Pose::CoordPositionAdd(...)
    }

    /// \brief Constructor that initializes each value, [w, x, y, z], of
    /// the quaternion. This constructor does not normalize the
    /// quaternion.
    /// \param[in] _w W param
    /// \param[in] _x X param
    /// \param[in] _y Y param
    /// \param[in] _z Z param
    public: constexpr Quaternion(const T &_w, const T &_x, const T &_y,
                                 const T &_z)
    : qw(_w), qx(_x), qy(_y), qz(_z)
    {}

    /// \brief Construct a Quaternion from Euler angles, in radians. This
    /// constructor normalizes the quaternion.
    /// \param[in] _roll  Roll radians.
    /// \param[in] _pitch Pitch radians.
    /// \param[in] _yaw   Yaw radians.
    /// \sa SetFromEuler(T, T, T)
    public: Quaternion(const T &_roll, const T &_pitch, const T &_yaw)
    {
      this->SetFromEuler(Vector3<T>(_roll, _pitch, _yaw));
    }

    /// \brief Constructor from an axis and angle. This constructor
    /// normalizes the quaternion.
    /// \param[in] _axis The rotation axis.
    /// \param[in] _angle The rotation angle in radians.
    public: Quaternion(const Vector3<T> &_axis, const T &_angle)
    {
      this->SetFromAxisAngle(_axis, _angle);
    }

    /// \brief Construct a Quaternion from Euler angles, in radians. This
    /// constructor normalizes the quaternion.
    /// \param[in] _rpy Euler angles in radians.
    public: explicit Quaternion(const Vector3<T> &_rpy)
    {
      this->SetFromEuler(_rpy);
    }

    /// \brief Construct from rotation matrix. This constructor does not
    /// normalize the quaternion.
    /// \param[in] _mat Rotation matrix (must be orthogonal, the function
    ///                 doesn't check it)
    public: explicit Quaternion(const Matrix3<T> &_mat)
    {
      this->SetFromMatrix(_mat);
    }

    /// \brief Invert the quaternion. The quaternion is first normalized,
    /// then inverted.
    public: void Invert()
    {
      this->Normalize();
      // this->qw = this->qw;
      this->qx = -this->qx;
      this->qy = -this->qy;
      this->qz = -this->qz;
    }

    /// \brief Get the inverse of this quaternion
    /// \return Inverse quaternion
    public: inline Quaternion<T> Inverse() const
    {
      T s = 0;
      Quaternion<T> q(this->qw, this->qx, this->qy, this->qz);

      // use s to test if quaternion is valid
      s = q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz;

      if (equal<T>(s, static_cast<T>(0)))
      {
        q.qw = 1.0;
        q.qx = 0.0;
        q.qy = 0.0;
        q.qz = 0.0;
      }
      else
      {
        // deal with non-normalized quaternion
        // div by s so q * qinv = identity
        q.qw =  q.qw / s;
        q.qx = -q.qx / s;
        q.qy = -q.qy / s;
        q.qz = -q.qz / s;
      }
      return q;
    }

    /// \brief Return the logarithm
    ///
    /// If q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x, y, z) is unit length,
    /// then log(q) = A*(x*i+y*j+z*k).  If sin(A) is near zero, use log(q) =
    /// sin(A)*(x*i+y*j+z*k) since sin(A)/A has limit 1.
    ///
    /// \return The log.
    public: Quaternion<T> Log() const
    {
      Quaternion<T> result;
      result.qw = 0.0;

      if (std::abs(this->qw) < 1.0)
      {
        T fAngle = acos(this->qw);
        T fSin = sin(fAngle);
        if (std::abs(fSin) >= 1e-3)
        {
          T fCoeff = fAngle/fSin;
          result.qx = fCoeff*this->qx;
          result.qy = fCoeff*this->qy;
          result.qz = fCoeff*this->qz;
          return result;
        }
      }

      result.qx = this->qx;
      result.qy = this->qy;
      result.qz = this->qz;

      return result;
    }

    /// \brief Return the exponent.
    ///
    /// If q = A*(x*i+y*j+z*k) where (x, y, z) is unit length, then
    /// exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k).  If sin(A) is near zero,
    /// use exp(q) = cos(A)+A*(x*i+y*j+z*k) since A/sin(A) has limit 1.
    ///
    /// \return The exponent.
    public: Quaternion<T> Exp() const
    {
      T fAngle = sqrt(this->qx*this->qx+
          this->qy*this->qy+this->qz*this->qz);
      T fSin = sin(fAngle);

      Quaternion<T> result;
      result.qw = cos(fAngle);

      if (std::abs(fSin) >= 1e-3)
      {
        T fCoeff = fSin/fAngle;
        result.qx = fCoeff*this->qx;
        result.qy = fCoeff*this->qy;
        result.qz = fCoeff*this->qz;
      }
      else
      {
        result.qx = this->qx;
        result.qy = this->qy;
        result.qz = this->qz;
      }

      return result;
    }

    /// \brief Normalize the quaternion.
    public: void Normalize()
    {
      T s = 0;

      s = T(sqrt(this->qw * this->qw + this->qx * this->qx +
          this->qy * this->qy + this->qz * this->qz));

      if (equal<T>(s, static_cast<T>(0)))
      {
        this->qw = T(1.0);
        this->qx = T(0.0);
        this->qy = T(0.0);
        this->qz = T(0.0);
      }
      else
      {
        this->qw /= s;
        this->qx /= s;
        this->qy /= s;
        this->qz /= s;
      }
    }

    /// \brief Gets a normalized version of this quaternion
    /// \return a normalized quaternion
    public: Quaternion<T> Normalized() const
    {
      Quaternion<T> result = *this;
      result.Normalize();
      return result;
    }

    /// \brief Set the quaternion from an axis and angle.
    /// \param[in] _ax X axis
    /// \param[in] _ay Y axis
    /// \param[in] _az Z axis
    /// \param[in] _aa Angle in radians
    public: void SetFromAxisAngle(T _ax, T _ay, T _az, T _aa)
    {
      T l;

      l = _ax * _ax + _ay * _ay + _az * _az;

      if (equal<T>(l, static_cast<T>(0)))
      {
        this->qw = 1;
        this->qx = 0;
        this->qy = 0;
        this->qz = 0;
      }
      else
      {
        _aa *= 0.5;
        l = sin(_aa) / sqrt(l);
        this->qw = cos(_aa);
        this->qx = _ax * l;
        this->qy = _ay * l;
        this->qz = _az * l;
      }

      this->Normalize();
    }

    /// \brief Set the quaternion from an axis and angle
    /// \param[in] _axis Axis
    /// \param[in] _a Angle in radians
    public: void SetFromAxisAngle(const Vector3<T> &_axis, T _a)
    {
      this->SetFromAxisAngle(_axis.X(), _axis.Y(), _axis.Z(), _a);
    }

    /// \brief Set this quaternion from 4 floating numbers
    /// \param[in] _w w
    /// \param[in] _x x
    /// \param[in] _y y
    /// \param[in] _z z
    public: void Set(T _w, T _x, T _y, T _z)
    {
      this->qw = _w;
      this->qx = _x;
      this->qy = _y;
      this->qz = _z;
    }

    /// \brief Set the quaternion from Euler angles. The order of operations
    /// is roll, pitch, yaw around a fixed body frame axis
    /// (the original frame of the object before rotation is applied).
    /// Roll is a rotation about x, pitch is about y, yaw is about z.
    /// \param[in] _vec Euler angles in radians.
    public: void SetFromEuler(const Vector3<T> &_vec)
    {
      this->SetFromEuler(_vec.X(), _vec.Y(), _vec.Z());
    }

    /// \brief Set the quaternion from Euler angles.
    /// \param[in] _roll Roll angle in radians.
    /// \param[in] _pitch Pitch angle in radians.
    /// \param[in] _yaw Yaw angle in radians.
    public: void SetFromEuler(T _roll, T _pitch, T _yaw)
    {
      T phi, the, psi;

      phi = _roll / T(2.0);
      the = _pitch / T(2.0);
      psi = _yaw / T(2.0);

      this->qw = T(cos(phi) * cos(the) * cos(psi) +
        sin(phi) * sin(the) * sin(psi));
      this->qx = T(sin(phi) * cos(the) * cos(psi) -
        cos(phi) * sin(the) * sin(psi));
      this->qy = T(cos(phi) * sin(the) * cos(psi) +
        sin(phi) * cos(the) * sin(psi));
      this->qz = T(cos(phi) * cos(the) * sin(psi) -
        sin(phi) * sin(the) * cos(psi));

      this->Normalize();
    }

    /// \brief Return the rotation in Euler angles, in radians.
    /// \return This quaternion as Euler angles.
    public: Vector3<T> Euler() const
    {
      Vector3<T> vec;

      T tol = static_cast<T>(1e-15);

      Quaternion<T> copy = *this;
      T squ;
      T sqx;
      T sqy;
      T sqz;

      copy.Normalize();

      squ = copy.qw * copy.qw;
      sqx = copy.qx * copy.qx;
      sqy = copy.qy * copy.qy;
      sqz = copy.qz * copy.qz;

      // Pitch
      T sarg = -2 * (copy.qx*copy.qz - copy.qw * copy.qy);
      if (sarg <= T(-1.0))
      {
        vec.Y(T(-0.5*GZ_PI));
      }
      else if (sarg >= T(1.0))
      {
        vec.Y(T(0.5*GZ_PI));
      }
      else
      {
        vec.Y(T(asin(sarg)));
      }

      // If the pitch angle is PI/2 or -PI/2, we can only compute
      // the sum roll + yaw.  However, any combination that gives
      // the right sum will produce the correct orientation, so we
      // set yaw = 0 and compute roll.
      // pitch angle is PI/2
      if (std::abs(sarg - 1) < tol)
      {
        vec.Z(0);
        vec.X(T(atan2(2 * (copy.qx*copy.qy - copy.qz*copy.qw),
                    squ - sqx + sqy - sqz)));
      }
      // pitch angle is -PI/2
      else if (std::abs(sarg + 1) < tol)
      {
        vec.Z(0);
        vec.X(T(atan2(-2 * (copy.qx*copy.qy - copy.qz*copy.qw),
                     squ - sqx + sqy - sqz)));
      }
      else
      {
        // Roll
        vec.X(T(atan2(2 * (copy.qy*copy.qz + copy.qw*copy.qx),
                    squ - sqx - sqy + sqz)));

        // Yaw
        vec.Z(T(atan2(2 * (copy.qx*copy.qy + copy.qw*copy.qz),
                    squ + sqx - sqy - sqz)));
      }

      return vec;
    }

    /// \brief Convert Euler angles to a quaternion.
    /// \param[in] _vec The vector of angles, in radians, to convert.
    /// \return The resulting quaternion
    public: static Quaternion<T> EulerToQuaternion(const Vector3<T> &_vec)
    {
      Quaternion<T> result;
      result.SetFromEuler(_vec);
      return result;
    }

    /// \brief Convert Euler angles, in radians, to a quaternion.
    /// \param[in] _x rotation along x in radians
    /// \param[in] _y rotation along y in radians
    /// \param[in] _z rotation along z in radians
    /// \return The resulting quaternion.
    public: static Quaternion<T> EulerToQuaternion(T _x, T _y, T _z)
    {
      return EulerToQuaternion(Vector3<T>(_x, _y, _z));
    }

    /// \brief Get the Euler roll angle in radians.
    /// \return The roll component.
    public: T Roll() const
    {
      return this->Euler().X();
    }

    /// \brief Get the Euler pitch angle in radians.
    /// \return The pitch component.
    public: T Pitch() const
    {
      return this->Euler().Y();
    }

    /// \brief Get the Euler yaw angle in radians.
    /// \return The yaw component.
    public: T Yaw() const
    {
      return this->Euler().Z();
    }

    /// \brief Convert this quaternion to an axis and angle.
    /// \param[out] _axis Rotation axis.
    /// \param[out] _angle CCW angle in radians.
    public: void AxisAngle(Vector3<T> &_axis, T &_angle) const
    {
      T sqLen = this->qx*this->qx + this->qy*this->qy + this->qz*this->qz;
      if (equal<T>(sqLen, static_cast<T>(0), static_cast<T>(1e-12)))
      {
        _angle = 0.0;
        _axis.Set(1, 0, 0);
      }
      else
      {
        _angle = 2.0 * acos(this->qw);
        T invLen =  1.0 / sqrt(sqLen);
        _axis.Set(this->qx*invLen, this->qy*invLen, this->qz*invLen);
      }
    }

    /// \brief Set from a rotation matrix.
    /// \param[in] _mat Rotation matrix (must be orthogonal, the function
    ///                 doesn't check it).
    ///
    /// Implementation inspired by
    /// http://www.euclideanspace.com/maths/geometry/rotations/
    /// conversions/matrixToQuaternion/
    public: void SetFromMatrix(const Matrix3<T> &_mat)
    {
      const T trace = _mat(0, 0) + _mat(1, 1) + _mat(2, 2);
      if (trace > 0.0000001)
      {
        qw = sqrt(1 + trace) / 2;
        const T s = 1.0 / (4 * qw);
        qx = (_mat(2, 1) - _mat(1, 2)) * s;
        qy = (_mat(0, 2) - _mat(2, 0)) * s;
        qz = (_mat(1, 0) - _mat(0, 1)) * s;
      }
      else if (_mat(0, 0) > _mat(1, 1) && _mat(0, 0) > _mat(2, 2))
      {
        qx = sqrt(1.0 + _mat(0, 0) - _mat(1, 1) - _mat(2, 2)) / 2;
        const T s = 1.0 / (4 * qx);
        qw = (_mat(2, 1) - _mat(1, 2)) * s;
        qy = (_mat(1, 0) + _mat(0, 1)) * s;
        qz = (_mat(0, 2) + _mat(2, 0)) * s;
      }
      else if (_mat(1, 1) > _mat(2, 2))
      {
        qy = sqrt(1.0 - _mat(0, 0) + _mat(1, 1) - _mat(2, 2)) / 2;
        const T s = 1.0 / (4 * qy);
        qw = (_mat(0, 2) - _mat(2, 0)) * s;
        qx = (_mat(0, 1) + _mat(1, 0)) * s;
        qz = (_mat(1, 2) + _mat(2, 1)) * s;
      }
      else
      {
        qz = sqrt(1.0 - _mat(0, 0) - _mat(1, 1) + _mat(2, 2)) / 2;
        const T s = 1.0 / (4 * qz);
        qw = (_mat(1, 0) - _mat(0, 1)) * s;
        qx = (_mat(0, 2) + _mat(2, 0)) * s;
        qy = (_mat(1, 2) + _mat(2, 1)) * s;
      }
    }

    /// \brief Set this quaternion to represent rotation from
    /// vector _v1 to vector _v2, so that
    /// _v2.Normalize() == this * _v1.Normalize() holds.
    ///
    /// \param[in] _v1 The first vector.
    /// \param[in] _v2 The second vector.
    ///
    /// Implementation inspired by
    /// http://stackoverflow.com/a/11741520/1076564
    public: void SetFrom2Axes(const Vector3<T> &_v1,
                const Vector3<T> &_v2)
    {
      // generally, we utilize the fact that a quat (w, x, y, z) represents
      // rotation of angle 2*w about axis (x, y, z)
      //
      // so we want to take get a vector half-way between no rotation and the
      // double rotation, which is
      // [ (1, (0, 0, 0)) + (_v1 dot _v2, _v1 x _v2) ] / 2
      // if _v1 and _v2 are unit quaternions
      //
      // since we normalize the result anyway, we can omit the division,
      // getting the result:
      // [ (1, (0, 0, 0)) + (_v1 dot _v2, _v1 x _v2) ].Normalized()
      //
      // if _v1 and _v2 are not normalized, the magnitude (1 + _v1 dot _v2)
      // is multiplied by k = norm(_v1)*norm(_v2)

      const T kCosTheta = _v1.Dot(_v2);
      const T k = sqrt(_v1.SquaredLength() * _v2.SquaredLength());

      if (fabs(kCosTheta/k + 1) < 1e-6)
      {
        // the vectors are opposite
        // any vector orthogonal to _v1
        Vector3<T> other;
        {
          const Vector3<T> _v1Abs(_v1.Abs());
          if (_v1Abs.X() < _v1Abs.Y())
          {
            if (_v1Abs.X() < _v1Abs.Z())
            {
              other.Set(1, 0, 0);
            }
            else
            {
              other.Set(0, 0, 1);
            }
          }
          else
          {
            if (_v1Abs.Y() < _v1Abs.Z())
            {
              other.Set(0, 1, 0);
            }
            else
            {
              other.Set(0, 0, 1);
            }
          }
        }

        const Vector3<T> axis(_v1.Cross(other).Normalize());

        qw = 0;
        qx = axis.X();
        qy = axis.Y();
        qz = axis.Z();
      }
      else
      {
        // the vectors are in general position
        const Vector3<T> axis(_v1.Cross(_v2));
        qw = kCosTheta + k;
        qx = axis.X();
        qy = axis.Y();
        qz = axis.Z();
        this->Normalize();
      }
    }

    /// \brief Scale this quaternion.
    /// \param[in] _scale Amount to scale this quaternion
    public: void Scale(T _scale)
    {
      Vector3<T> axis;
      T angle;

      // Convert to axis-and-angle
      this->AxisAngle(axis, angle);
      angle *= _scale;

      this->SetFromAxisAngle(axis.X(), axis.Y(), axis.Z(), angle);
    }

    /// \brief Addition operator.
    /// \param[in] _qt Quaternion for addition.
    /// \return This quaternion + _qt.
    public: Quaternion<T> operator+(const Quaternion<T> &_qt) const
    {
      Quaternion<T> result(this->qw + _qt.qw, this->qx + _qt.qx,
                           this->qy + _qt.qy, this->qz + _qt.qz);
      return result;
    }

    /// \brief Addition set operator.
    /// \param[in] _qt Quaternion for addition.
    /// \return This quaternion + qt.
    public: Quaternion<T> operator+=(const Quaternion<T> &_qt)
    {
      *this = *this + _qt;

      return *this;
    }

    /// \brief Subtraction operator.
    /// \param[in] _qt Quaternion to subtract.
    /// \return This quaternion - _qt
    public: Quaternion<T> operator-(const Quaternion<T> &_qt) const
    {
      Quaternion<T> result(this->qw - _qt.qw, this->qx - _qt.qx,
                     this->qy - _qt.qy, this->qz - _qt.qz);
      return result;
    }

    /// \brief Subtraction set operator.
    /// \param[in] _qt Quaternion for subtraction.
    /// \return This quaternion - qt.
    public: Quaternion<T> operator-=(const Quaternion<T> &_qt)
    {
      *this = *this - _qt;
      return *this;
    }

    /// \brief Multiplication operator.
    /// \param[in] _q Quaternion for multiplication.
    /// \return This quaternion multiplied by the parameter.
    public: inline Quaternion<T> operator*(const Quaternion<T> &_q) const
            {
              return Quaternion<T>(
                this->qw*_q.qw-this->qx*_q.qx-this->qy*_q.qy-this->qz*_q.qz,
                this->qw*_q.qx+this->qx*_q.qw+this->qy*_q.qz-this->qz*_q.qy,
                this->qw*_q.qy-this->qx*_q.qz+this->qy*_q.qw+this->qz*_q.qx,
                this->qw*_q.qz+this->qx*_q.qy-this->qy*_q.qx+this->qz*_q.qw);
            }

    /// \brief Multiplication operator by a scalar.
    /// \param[in] _f Factor.
    /// \return Quaternion multiplied by the scalar.
    public: Quaternion<T> operator*(const T &_f) const
    {
      return Quaternion<T>(this->qw*_f, this->qx*_f,
                           this->qy*_f, this->qz*_f);
    }

    /// \brief Multiplication set operator.
    /// \param[in] _qt Quaternion<T> for multiplication.
    /// \return This quaternion multiplied by the parameter.
    public: Quaternion<T> operator*=(const Quaternion<T> &_qt)
    {
      *this = *this * _qt;
      return *this;
    }

    /// \brief Vector3 multiplication operator.
    /// \param[in] _v vector to multiply.
    /// \return The result of the vector multiplication.
    public: Vector3<T> operator*(const Vector3<T> &_v) const
    {
      Vector3<T> uv, uuv;
      Vector3<T> qvec(this->qx, this->qy, this->qz);
      uv = qvec.Cross(_v);
      uuv = qvec.Cross(uv);
      uv *= (2.0f * this->qw);
      uuv *= 2.0f;

      return _v + uv + uuv;
    }

    /// \brief Equality comparison operator. A tolerance of 0.001 is used
    /// with the gz::math::equal function for each component of the
    /// quaternions.
    /// \param[in] _qt Quaternion<T> for comparison.
    /// \return True if each component of both quaternions is within the
    /// tolerance of 0.001 of its counterpart.
    public: bool operator==(const Quaternion<T> &_qt) const
    {
      return this->Equal(_qt, static_cast<T>(0.001));
    }

    /// \brief Not equal to operator. A tolerance of 0.001 is used
    /// with the gz::math::equal function for each component of the
    /// quaternions.
    /// \param[in] _qt Quaternion for comparison.
    /// \return True if any component of both quaternions is not within
    /// the tolerance of 0.001 of its counterpart.
    public: bool operator!=(const Quaternion<T> &_qt) const
    {
      return !(*this == _qt);
    }

    /// \brief Unary minus operator.
    /// \return Negation of each component of this quaternion.
    public: Quaternion<T> operator-() const
    {
      return Quaternion<T>(-this->qw, -this->qx, -this->qy, -this->qz);
    }

    /// \brief Rotate a vector using the quaternion.
    /// \param[in] _vec Vector to rotate.
    /// \return The rotated vector.
    public: inline Vector3<T> RotateVector(const Vector3<T> &_vec) const
    {
      Quaternion<T> tmp(static_cast<T>(0),
          _vec.X(), _vec.Y(), _vec.Z());
      tmp = (*this) * (tmp * this->Inverse());
      return Vector3<T>(tmp.qx, tmp.qy, tmp.qz);
    }

    /// \brief Get the reverse rotation of a vector by this quaternion.
    /// \param[in] _vec The vector.
    /// \return The reversed vector.
    public: Vector3<T> RotateVectorReverse(const Vector3<T> &_vec) const
    {
      Quaternion<T> tmp(0.0, _vec.X(), _vec.Y(), _vec.Z());

      tmp =  this->Inverse() * (tmp * (*this));

      return Vector3<T>(tmp.qx, tmp.qy, tmp.qz);
    }

    /// \brief See if a quaternion is finite (e.g., not nan).
    /// \return True if quaternion is finite.
    public: bool IsFinite() const
    {
      // std::isfinite works with floating point values, need to explicit
      // cast to avoid ambiguity in vc++.
      return std::isfinite(static_cast<double>(this->qw)) &&
             std::isfinite(static_cast<double>(this->qx)) &&
             std::isfinite(static_cast<double>(this->qy)) &&
             std::isfinite(static_cast<double>(this->qz));
    }

    /// \brief Correct any nan values in this quaternion.
    public: inline void Correct()
    {
      // std::isfinite works with floating point values, need to explicit
      // cast to avoid ambiguity in vc++.
      if (!std::isfinite(static_cast<double>(this->qx)))
        this->qx = 0;
      if (!std::isfinite(static_cast<double>(this->qy)))
        this->qy = 0;
      if (!std::isfinite(static_cast<double>(this->qz)))
        this->qz = 0;
      if (!std::isfinite(static_cast<double>(this->qw)))
        this->qw = 1;

      if (equal(this->qw, static_cast<T>(0)) &&
          equal(this->qx, static_cast<T>(0)) &&
          equal(this->qy, static_cast<T>(0)) &&
          equal(this->qz, static_cast<T>(0)))
      {
        this->qw = 1;
      }
    }

    /// \brief Return the X axis.
    /// \return the X axis of the vector.
    public: Vector3<T> XAxis() const
    {
      T fTy  = 2.0f*this->qy;
      T fTz  = 2.0f*this->qz;

      T fTwy = fTy*this->qw;
      T fTwz = fTz*this->qw;
      T fTxy = fTy*this->qx;
      T fTxz = fTz*this->qx;
      T fTyy = fTy*this->qy;
      T fTzz = fTz*this->qz;

      return Vector3<T>(1.0f-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);
    }

    /// \brief Return the Y axis.
    /// \return the Y axis of the vector.
    public: Vector3<T> YAxis() const
    {
      T fTx  = 2.0f*this->qx;
      T fTy  = 2.0f*this->qy;
      T fTz  = 2.0f*this->qz;
      T fTwx = fTx*this->qw;
      T fTwz = fTz*this->qw;
      T fTxx = fTx*this->qx;
      T fTxy = fTy*this->qx;
      T fTyz = fTz*this->qy;
      T fTzz = fTz*this->qz;

      return Vector3<T>(fTxy-fTwz, 1.0f-(fTxx+fTzz), fTyz+fTwx);
    }

    /// \brief Return the Z axis.
    /// \return the Z axis of the vector.
    public: Vector3<T> ZAxis() const
    {
      T fTx  = 2.0f*this->qx;
      T fTy  = 2.0f*this->qy;
      T fTz  = 2.0f*this->qz;
      T fTwx = fTx*this->qw;
      T fTwy = fTy*this->qw;
      T fTxx = fTx*this->qx;
      T fTxz = fTz*this->qx;
      T fTyy = fTy*this->qy;
      T fTyz = fTz*this->qy;

      return Vector3<T>(fTxz+fTwy, fTyz-fTwx, 1.0f-(fTxx+fTyy));
    }

    /// \brief Round all values to _precision decimal places.
    /// \param[in] _precision the precision.
    public: void Round(int _precision)
    {
      this->qx = precision(this->qx, _precision);
      this->qy = precision(this->qy, _precision);
      this->qz = precision(this->qz, _precision);
      this->qw = precision(this->qw, _precision);
    }

    /// \brief Get the dot product of this quaternion with the give _q
    /// quaternion.
    /// \param[in] _q The other quaternion.
    /// \return The dot product.
    public: T Dot(const Quaternion<T> &_q) const
    {
      return this->qw*_q.qw + this->qx * _q.qx +
             this->qy*_q.qy + this->qz*_q.qz;
    }

    /// \brief Spherical quadratic interpolation
    /// given the ends and an interpolation parameter between 0 and 1.
    /// \param[in] _fT the interpolation parameter.
    /// \param[in] _rkP The beginning quaternion.
    /// \param[in] _rkA First intermediate quaternion.
    /// \param[in] _rkB Second intermediate quaternion.
    /// \param[in] _rkQ The end quaternion.
    /// \param[in] _shortestPath When true, the rotation may be inverted to
    /// get to minimize rotation.
    /// \return The result of the quadratic interpolation.
    public: static Quaternion<T> Squad(T _fT,
                const Quaternion<T> &_rkP, const Quaternion<T> &_rkA,
                const Quaternion<T> &_rkB, const Quaternion<T> &_rkQ,
                bool _shortestPath = false)
    {
      T fSlerpT = 2.0f*_fT*(1.0f-_fT);
      Quaternion<T> kSlerpP = Slerp(_fT, _rkP, _rkQ, _shortestPath);
      Quaternion<T> kSlerpQ = Slerp(_fT, _rkA, _rkB);
      return Slerp(fSlerpT, kSlerpP, kSlerpQ);
    }

    /// \brief Spherical linear interpolation between 2 quaternions,
    /// given the ends and an interpolation parameter between 0 and 1.
    /// \param[in] _fT The interpolation parameter.
    /// \param[in] _rkP The beginning quaternion.
    /// \param[in] _rkQ The end quaternion.
    /// \param[in] _shortestPath When true, the rotation may be inverted to
    /// get to minimize rotation.
    /// \return The result of the linear interpolation.
    public: static Quaternion<T> Slerp(T _fT,
                const Quaternion<T> &_rkP, const Quaternion<T> &_rkQ,
                bool _shortestPath = false)
    {
      T fCos = _rkP.Dot(_rkQ);
      Quaternion<T> rkT;

      // Do we need to invert rotation?
      if (fCos < 0.0f && _shortestPath)
      {
        fCos = -fCos;
        rkT = -_rkQ;
      }
      else
      {
        rkT = _rkQ;
      }

      if (std::abs(fCos) < 1 - 1e-03)
      {
        // Standard case (slerp)
        T fSin = sqrt(1 - (fCos*fCos));
        T fAngle = atan2(fSin, fCos);
        // FIXME: should check if (std::abs(fSin) >= 1e-3)
        T fInvSin = 1.0f / fSin;
        T fCoeff0 = sin((1.0f - _fT) * fAngle) * fInvSin;
        T fCoeff1 = sin(_fT * fAngle) * fInvSin;
        return _rkP * fCoeff0 + rkT * fCoeff1;
      }
      else
      {
        // There are two situations:
        // 1. "rkP" and "rkQ" are very close (fCos ~= +1),
        // so we can do a linear interpolation safely.
        // 2. "rkP" and "rkQ" are almost inverse of each
        // other (fCos ~= -1), there
        // are an infinite number of possibilities interpolation.
        // but we haven't have method to fix this case, so just use
        // linear interpolation here.
        Quaternion<T> t = _rkP * (1.0f - _fT) + rkT * _fT;
        // taking the complement requires renormalisation
        t.Normalize();
        return t;
      }
    }

    /// \brief Integrate quaternion for constant angular velocity vector
    /// along specified interval `_deltaT`.
    /// Implementation based on:
    /// http://physicsforgames.blogspot.com/2010/02/quaternions.html
    /// \param[in] _angularVelocity Angular velocity vector, specified in
    /// same reference frame as base of this quaternion.
    /// \param[in] _deltaT Time interval in seconds to integrate over.
    /// \return Quaternion at integrated configuration.
    public: Quaternion<T> Integrate(const Vector3<T> &_angularVelocity,
                                    const T _deltaT) const
    {
      Quaternion<T> deltaQ;
      Vector3<T> theta = _angularVelocity * _deltaT / 2;
      T thetaMagSq = theta.SquaredLength();
      T s;
      if (thetaMagSq * thetaMagSq / 24.0 < MIN_D)
      {
        deltaQ.W() = 1.0 - thetaMagSq / 2.0;
        s = 1.0 - thetaMagSq / 6.0;
      }
      else
      {
        double thetaMag = sqrt(thetaMagSq);
        deltaQ.W() = cos(thetaMag);
        s = sin(thetaMag) / thetaMag;
      }
      deltaQ.X() = theta.X() * s;
      deltaQ.Y() = theta.Y() * s;
      deltaQ.Z() = theta.Z() * s;
      return deltaQ * (*this);
    }

    /// \brief Get the w component.
    /// \return The w quaternion component.
    public: inline T W() const
    {
      return this->qw;
    }

    /// \brief Get the x component.
    /// \return The x quaternion component.
    public: inline T X() const
    {
      return this->qx;
    }

    /// \brief Get the y component.
    /// \return The y quaternion component.
    public: inline T Y() const
    {
      return this->qy;
    }

    /// \brief Get the z component.
    /// \return The z quaternion component.
    public: inline T Z() const
    {
      return this->qz;
    }

    /// \brief Get a mutable w component.
    /// \return The w quaternion component.
    public: inline T &W()
    {
      return this->qw;
    }

    /// \brief Get a mutable x component.
    /// \return The x quaternion component.
    public: inline T &X()
    {
      return this->qx;
    }

    /// \brief Get a mutable y component.
    /// \return The y quaternion component.
    public: inline T &Y()
    {
      return this->qy;
    }

    /// \brief Get a mutable z component.
    /// \return The z quaternion component.
    public: inline T &Z()
    {
      return this->qz;
    }

    /// \brief Set the x component.
    /// \param[in] _v The new value for the x quaternion component.
    public: inline void SetX(T _v)
    {
      this->qx = _v;
    }

    /// \brief Set the y component.
    /// \param[in] _v The new value for the y quaternion component.
    public: inline void SetY(T _v)
    {
      this->qy = _v;
    }


    /// \brief Set the z component.
    /// \param[in] _v The new value for the z quaternion component.
    public: inline void SetZ(T _v)
    {
      this->qz = _v;
    }

    /// \brief Set the w component.
    /// \param[in] _v The new value for the w quaternion component.
    public: inline void SetW(T _v)
    {
      this->qw = _v;
    }

    /// \brief Stream insertion operator
    /// \param[in, out] _out output stream
    /// \param[in] _q quaternion to output
    /// \return the stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                const gz::math::Quaternion<T> &_q)
    {
      Vector3<T> v(_q.Euler());
      _out << v;
      return _out;
    }

    /// \brief Stream extraction operator
    /// \param[in, out] _in input stream
    /// \param[out] _q Quaternion<T> to read values into
    /// \return The istream
    public: friend std::istream &operator>>(std::istream &_in,
        gz::math::Quaternion<T> &_q)
    {
      Angle roll, pitch, yaw;

      // Skip white spaces
      _in.setf(std::ios_base::skipws);
      _in >> roll >> pitch >> yaw;

      if (!_in.fail())
      {
        _q.SetFromEuler(Vector3<T>(*roll, *pitch, *yaw));
      }

      return _in;
    }

    /// \brief Equality comparison test with a tolerance parameter.
    /// The tolerance is used with the gz::math::equal function for
    /// each component of the quaternions.
    /// \param[in] _q The quaternion to compare against.
    /// \param[in] _tol equality tolerance.
    /// \return True if the elements of the quaternions are equal within
    /// the tolerance specified by _tol.
    public: bool Equal(const Quaternion &_q, const T &_tol) const
    {
      return equal(this->qx, _q.qx, _tol) &&
             equal(this->qy, _q.qy, _tol) &&
             equal(this->qz, _q.qz, _tol) &&
             equal(this->qw, _q.qw, _tol);
    }

    /// \brief w value of the quaternion
    private: T qw;

    /// \brief x value of the quaternion
    private: T qx;

    /// \brief y value of the quaternion
    private: T qy;

    /// \brief z value of the quaternion
    private: T qz;
  };

  namespace detail {

    template<typename T> constexpr Quaternion<T>
    gQuaternionIdentity(1, 0, 0, 0);

    template<typename T> constexpr Quaternion<T>
    gQuaternionZero(0, 0, 0, 0);

  }  // namespace detail

  template<typename T> const Quaternion<T>
  &Quaternion<T>::Identity = detail::gQuaternionIdentity<T>;

  template<typename T> const Quaternion<T>
  &Quaternion<T>::Zero = detail::gQuaternionZero<T>;

  /// typedef Quaternion<double> as Quaterniond
  typedef Quaternion<double> Quaterniond;

  /// typedef Quaternion<float> as Quaternionf
  typedef Quaternion<float> Quaternionf;
  }  // namespace GZ_MATH_VERSION_NAMESPACE
}  // namespace gz::math
#endif  // GZ_MATH_QUATERNION_HH_
