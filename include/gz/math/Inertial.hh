/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_INERTIAL_HH_
#define IGNITION_MATH_INERTIAL_HH_

#include <ignition/math/config.hh>
#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Pose3.hh"

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Inertial Inertial.hh ignition/math/Inertial.hh
    /// \brief The Inertial object provides a representation for the mass and
    /// inertia matrix of a body B. The components of the inertia matrix are
    /// expressed in what we call the "inertial" frame Bi of the body, i.e.
    /// the frame in which these inertia components are measured. The inertial
    /// frame Bi must be located at the center of mass of the body, but not
    /// necessarily aligned with the body’s frame. In addition, this class
    /// allows users to specify a frame F for these inertial properties by
    /// specifying the pose X_FBi of the inertial frame Bi in the
    /// inertial object frame F.
    ///
    /// For information about the X_FBi notation, see
    /// http://drake.mit.edu/doxygen_cxx/group__multibody__spatial__pose.html
    template<typename T>
    class Inertial
    {
      /// \brief Default Constructor
      public: Inertial()
      {}

      /// \brief Constructs an inertial object from the mass matrix for a body
      /// B, about its center of mass Bcm, and expressed in a frame that we’ll
      /// call the "inertial" frame Bi, i.e. the frame in which the components
      /// of the mass matrix are specified (see this class’s documentation for
      /// details). The pose object specifies the pose X_FBi of the inertial
      /// frame Bi in the frame F of this inertial object
      /// (see class’s documentation).
      /// \param[in] _massMatrix Mass and inertia matrix.
      /// \param[in] _pose Pose of center of mass reference frame.
      public: Inertial(const MassMatrix3<T> &_massMatrix,
                       const Pose3<T> &_pose)
      : massMatrix(_massMatrix), pose(_pose)
      {}

      /// \brief Copy constructor.
      /// \param[in] _inertial Inertial element to copy
      public: Inertial(const Inertial<T> &_inertial)
      : massMatrix(_inertial.MassMatrix()), pose(_inertial.Pose())
      {}

      /// \brief Destructor.
      public: virtual ~Inertial() {}

      /// \brief Set the mass and inertia matrix.
      ///
      /// \param[in] _m New MassMatrix3 object.
      /// \param[in] _tolerance Tolerance is passed to
      /// MassMatrix3::IsValid and is the amount of error
      /// to accept when checking whether the MassMatrix3 _m is valid.
      /// Refer to MassMatrix3::Epsilon for detailed description of
      /// _tolerance.
      ///
      /// \return True if the MassMatrix3 is valid.
      public: bool SetMassMatrix(const MassMatrix3<T> &_m,
                  const T _tolerance = IGN_MASSMATRIX3_DEFAULT_TOLERANCE<T>)
      {
        this->massMatrix = _m;
        return this->massMatrix.IsValid(_tolerance);
      }

      /// \brief Get the mass and inertia matrix.
      /// \return The mass matrix about the body’s center of mass and
      /// expressed in the inertial frame Bi as defined by this class’s
      /// documentation
      public: const MassMatrix3<T> &MassMatrix() const
      {
        return this->massMatrix;
      }

      /// \brief Set the pose of the center of mass reference frame.
      /// \param[in] _pose New pose.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetPose(const Pose3<T> &_pose)
      {
        this->pose = _pose;
        return this->massMatrix.IsValid();
      }

      /// \brief Get the pose of the center of mass reference frame.
      /// \return The pose of the inertial frame Bi in the frame F of this
      /// Inertial object as defined by this class’s documentation.
      public: const Pose3<T> &Pose() const
      {
        return this->pose;
      }

      /// \copydoc Moi() const
      /// \deprecated See Matrix3<T> Moi() const
      public: Matrix3<T> IGN_DEPRECATED(5.0) MOI() const
      {
        return this->Moi();
      }

      /// \brief Get the moment of inertia matrix computer about the body's
      /// center of mass and expressed in this Inertial object’s frame F.
      /// \return The inertia matrix computed about the body’s center of
      /// mass and expressed in this Inertial object’s frame F, as defined
      /// in this class’s documentation.
      public: Matrix3<T> Moi() const
      {
        auto R = Matrix3<T>(this->pose.Rot());
        return R * this->massMatrix.Moi() * R.Transposed();
      }

      /// \brief Set the inertial pose rotation without affecting the
      /// MOI in the base coordinate frame.
      /// \param[in] _q New rotation for inertial pose.
      /// \return True if the MassMatrix3 is valid.
      public: bool SetInertialRotation(const Quaternion<T> &_q)
      {
        auto moi = this->Moi();
        this->pose.Rot() = _q;
        auto R = Matrix3<T>(_q);
        return this->massMatrix.SetMoi(R.Transposed() * moi * R);
      }

      /// \brief Set the MassMatrix rotation (eigenvectors of inertia matrix)
      /// without affecting the MOI in the base coordinate frame.
      /// Note that symmetries in inertia matrix may prevent the output of
      /// MassMatrix3::PrincipalAxesOffset to match this function's input _q,
      /// but it is guaranteed that the MOI in the base frame will not change.
      /// A negative value of _tol (such as -1e-6) can be passed to ensure
      /// that diagonal values are always sorted.
      /// \param[in] _q New rotation.
      /// \param[in] _tol Relative tolerance given by absolute value
      /// of _tol. This is passed to the MassMatrix3
      /// PrincipalMoments and PrincipalAxesOffset functions.
      /// \return True if the MassMatrix3 is valid.
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

      /// \brief Equal operator.
      /// \param[in] _inertial Inertial to copy.
      /// \return Reference to this object.
      public: Inertial &operator=(const Inertial<T> &_inertial)
      {
        this->massMatrix = _inertial.MassMatrix();
        this->pose = _inertial.Pose();

        return *this;
      }

      /// \brief Equality comparison operator.
      /// \param[in] _inertial Inertial to copy.
      /// \return true if each component is equal within a default tolerance,
      /// false otherwise
      public: bool operator==(const Inertial<T> &_inertial) const
      {
        return (this->pose == _inertial.Pose()) &&
               (this->massMatrix == _inertial.MassMatrix());
      }

      /// \brief Inequality test operator
      /// \param[in] _inertial Inertial<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const Inertial<T> &_inertial) const
      {
        return !(*this == _inertial);
      }

      /// \brief Adds inertial properties to current object.
      /// The mass, center of mass location, and inertia matrix are updated
      /// as long as the total mass is positive.
      /// \param[in] _inertial Inertial to add.
      /// \return Reference to this object.
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

      /// \brief Adds inertial properties to current object.
      /// The mass, center of mass location, and inertia matrix are updated
      /// as long as the total mass is positive.
      /// \param[in] _inertial Inertial to add.
      /// \return Sum of inertials as new object.
      public: const Inertial<T> operator+(const Inertial<T> &_inertial) const
      {
        return Inertial<T>(*this) += _inertial;
      }

      /// \brief Mass and inertia matrix of the object expressed in the
      /// center of mass reference frame.
      private: MassMatrix3<T> massMatrix;

      /// \brief Pose offset of center of mass reference frame relative
      /// to a base frame.
      private: Pose3<T> pose;
    };

    typedef Inertial<double> Inertiald;
    typedef Inertial<float> Inertialf;
    }
  }
}
#endif
