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

%module inertial
%{
#include <ignition/math/Inertial.hh>
#include <ignition/math/config.hh>
#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Pose3.hh"
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class Inertial
    {
      public: Inertial()
      {}

      public: Inertial(const MassMatrix3<T> &_massMatrix,
                       const Pose3<T> &_pose)
      : massMatrix(_massMatrix), pose(_pose)
      {}

      public: Inertial(const Inertial<T> &_inertial)
      : massMatrix(_inertial.MassMatrix()), pose(_inertial.Pose())
      {}

      public: virtual ~Inertial() {}

      public: bool SetMassMatrix(const MassMatrix3<T> &_m,
                  const T _tolerance = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>)
      {
        this->massMatrix = _m;
        return this->massMatrix.IsValid(_tolerance);
      }

      public: const MassMatrix3<T> &MassMatrix() const
      {
        return this->massMatrix;
      }

      public: bool SetPose(const Pose3<T> &_pose)
      {
        this->pose = _pose;
        return this->massMatrix.IsValid();
      }

      public: const Pose3<T> &Pose() const
      {
        return this->pose;
      }

      public: Matrix3<T> Moi() const
      {
        auto R = Matrix3<T>(this->pose.Rot());
        return R * this->massMatrix.Moi() * R.Transposed();
      }

      public: bool SetInertialRotation(const Quaternion<T> &_q)
      {
        auto moi = this->Moi();
        this->pose.Rot() = _q;
        auto R = Matrix3<T>(_q);
        return this->massMatrix.SetMoi(R.Transposed() * moi * R);
      }

      public: bool SetMassMatrixRotation(const Quaternion<T> &_q,
                                         const T _tol = 1e-6)
      {
        this->pose.Rot() *= this->MassMatrix().PrincipalAxesOffset(_tol) *
                            _q.Inverse();
        const auto moments = this->MassMatrix().PrincipalMoments(_tol);
        const auto diag = Matrix3<T>(
            moments[0], 0, 0,
            0, moments[1], 0,
            0, 0, moments[2]);
        const auto R = Matrix3<T>(_q);
        return this->massMatrix.SetMoi(R * diag * R.Transposed());
      }

      public: bool operator==(const Inertial<T> &_inertial) const
      {
        return (this->pose == _inertial.Pose()) &&
               (this->massMatrix == _inertial.MassMatrix());
      }

      public: bool operator!=(const Inertial<T> &_inertial) const
      {
        return !(*this == _inertial);
      }

      public: Inertial<T> &operator+=(const Inertial<T> &_inertial)
      {
        T m1 = this->massMatrix.Mass();
        T m2 = _inertial.MassMatrix().Mass();

        // Total mass
        T mass = m1 + m2;

        // Only continue if total mass is positive
        if (mass <= 0)
        {
          return *this;
        }

        auto com1 = this->Pose().Pos();
        auto com2 = _inertial.Pose().Pos();
        // New center of mass location in base frame
        auto com = (m1*com1 + m2*com2) / mass;

        // Components of new moment of inertia matrix
        Vector3<T> ixxyyzz;
        Vector3<T> ixyxzyz;
        // First add matrices in base frame
        {
          auto moi = this->Moi() + _inertial.Moi();
          ixxyyzz = Vector3<T>(moi(0, 0), moi(1, 1), moi(2, 2));
          ixyxzyz = Vector3<T>(moi(0, 1), moi(0, 2), moi(1, 2));
        }
        // Then account for parallel axis theorem
        {
          auto dc = com1 - com;
          ixxyyzz.X() += m1 * (std::pow(dc[1], 2) + std::pow(dc[2], 2));
          ixxyyzz.Y() += m1 * (std::pow(dc[2], 2) + std::pow(dc[0], 2));
          ixxyyzz.Z() += m1 * (std::pow(dc[0], 2) + std::pow(dc[1], 2));
          ixyxzyz.X() -= m1 * dc[0] * dc[1];
          ixyxzyz.Y() -= m1 * dc[0] * dc[2];
          ixyxzyz.Z() -= m1 * dc[1] * dc[2];
        }
        {
          auto dc = com2 - com;
          ixxyyzz.X() += m2 * (std::pow(dc[1], 2) + std::pow(dc[2], 2));
          ixxyyzz.Y() += m2 * (std::pow(dc[2], 2) + std::pow(dc[0], 2));
          ixxyyzz.Z() += m2 * (std::pow(dc[0], 2) + std::pow(dc[1], 2));
          ixyxzyz.X() -= m2 * dc[0] * dc[1];
          ixyxzyz.Y() -= m2 * dc[0] * dc[2];
          ixyxzyz.Z() -= m2 * dc[1] * dc[2];
        }
        this->massMatrix = MassMatrix3<T>(mass, ixxyyzz, ixyxzyz);
        this->pose = Pose3<T>(com, Quaternion<T>::Identity);

        return *this;
      }

      public: const Inertial<T> operator+(const Inertial<T> &_inertial) const
      {
        return Inertial<T>(*this) += _inertial;
      }
    };
    %template(Inertiald) Inertial<double>;

  }
}
