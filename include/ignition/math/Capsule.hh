/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_CAPSULE_HH_
#define IGNITION_MATH_CAPSULE_HH_

#include "ignition/math/MassMatrix3.hh"
#include "ignition/math/Material.hh"
#include "ignition/math/Quaternion.hh"

namespace ignition
{
  namespace math
  {
    // Foward declarations
    class CapsulePrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \class Capsule Capsule.hh ignition/math/Capsule.hh
    /// \brief A representation of a capsule or sphere-capped cylinder.
    ///
    /// The capsule class supports defining a capsule with a radius,
    /// length, rotational offset, and material properties. The shape is
    /// equivalent to a cylinder with capped with hemispheres. Radius and
    /// length are in meters. See Material for more on material properties.
    /// By default, a capsule's length is aligned with the Z axis. The
    /// rotational offset encodes a rotation from the z axis.
    template<typename Precision>
    class Capsule
    {
      /// \brief Default constructor. The default radius and length are both
      /// zero. The default rotational offset is
      /// Quaternion<Precision>::Identity.
      public: Capsule() = default;

      /// \brief Construct a capsule with a length, radius, and optionally
      /// a rotational offset.
      /// \param[in] _length Length of the capsule.
      /// \param[in] _radius Radius of the capsule.
      /// \param[in] _rotOffset Rotational offset of the capsule.
      public: Capsule(const Precision _length, const Precision _radius,
                  const Quaternion<Precision> &_rotOffset =
                  Quaternion<Precision>::Identity);

      /// \brief Construct a capsule with a length, radius, material and
      /// optionally a rotational offset.
      /// \param[in] _length Length of the capsule.
      /// \param[in] _radius Radius of the capsule.
      /// \param[in] _mat Material property for the capsule.
      /// \param[in] _rotOffset Rotational offset of the capsule.
      public: Capsule(const Precision _length, const Precision _radius,
                  const Material &_mat,
                  const Quaternion<Precision> &_rotOffset =
                  Quaternion<Precision>::Identity);

      /// \brief Destructor
      public: ~Capsule() = default;

      /// \brief Get the radius in meters.
      /// \return The radius of the capsule in meters.
      public: Precision Radius() const;

      /// \brief Set the radius in meters.
      /// \param[in] _radius The radius of the capsule in meters.
      public: void SetRadius(const Precision _radius);

      /// \brief Get the length in meters.
      /// \return The length of the capsule in meters.
      public: Precision Length() const;

      /// \brief Set the length in meters.
      /// \param[in] _length The length of the capsule in meters.
      public: void SetLength(const Precision _length);

      /// \brief Get the rotational offset. By default, a capsule's length
      /// is aligned with the Z axis. The rotational offset encodes
      /// a rotation from the z axis.
      /// \return The capsule's rotational offset.
      /// \sa void SetRotationalOffset(const Quaternion<Precision> &_rot)
      public: Quaternion<Precision> RotationalOffset() const;

      /// \brief Set the rotation offset.
      /// See Quaternion<Precision> RotationalOffset() for details on the
      /// rotational offset.
      /// \return The capsule's orientation.
      /// \sa Quaternion<Precision> RotationalOffset() const
      public: void SetRotationalOffset(
                  const Quaternion<Precision> &_rotOffset);

      /// \brief Set the length in meters.
      /// \param[in] _length The length of the capsule in meters.
      public: void SetLength(const Precision _length) const;

      /// \brief Get the material associated with this capsule.
      /// \return The material assigned to this capsule
      public: const Material &Mat() const;

      /// \brief Set the material associated with this capsule.
      /// \param[in] _mat The material assigned to this capsule
      public: void SetMat(const Material &_mat);

      /// \brief Get the mass matrix for this capsule. This function
      /// is only meaningful if the capsule's radius, length, and material
      /// have been set. Optionally, set the rotational offset.
      /// \param[out] _massMat The computed mass matrix will be stored
      /// here.
      /// \return False if computation of the mass matrix failed, which
      /// could be due to an invalid radius (<=0), length (<=0), or density
      /// (<=0).
      public: bool MassMatrix(MassMatrix3d &_massMat) const;

      /// \brief Check if this capsule is equal to the provided capsule.
      /// Radius, length, and material properties will be checked.
      public: bool operator==(const Capsule &_capsule) const;

      /// \brief Get the volume of the capsule in m^3.
      /// \return Volume of the capsule in m^3.
      public: Precision Volume() const;

      /// \brief Compute the capsule's density given a mass value. The
      /// capsule is assumed to be solid with uniform density. This
      /// function requires the capsule's radius and length to be set to
      /// values greater than zero. The Material of the capsule is ignored.
      /// \param[in] _mass Mass of the capsule, in kg. This value should be
      /// greater than zero.
      /// \return Density of the capsule in kg/m^3. A negative value is
      /// returned if radius, length or _mass is <= 0.
      public: Precision DensityFromMass(const Precision _mass) const;

      /// \brief Set the density of this capsule based on a mass value.
      /// Density is computed using
      /// Precision DensityFromMass(const Precision _mass) const. The
      /// capsule is assumed to be solid with uniform density. This
      /// function requires the capsule's radius and length to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the capsule, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// capsule's radius, length, or the _mass value are <= 0.
      /// \sa Precision DensityFromMass(const Precision _mass) const
      public: bool SetDensityFromMass(const Precision _mass);

      /// \brief Radius of the capsule.
      private: Precision radius = 0.0;

      /// \brief Length of the capsule.
      private: Precision length = 0.0;

      /// \brief the capsule's material.
      private: Material material;

      /// \brief Rotational offset.
      private: Quaternion<Precision> rotOffset =
               Quaternion<Precision>::Identity;
    };

    /// \typedef Capsule<int> Capsulei
    /// \brief Capsule with integer precision.
    typedef Capsule<int> Capsulei;

    /// \typedef Capsule<double> Capsuled
    /// \brief Capsule with double precision.
    typedef Capsule<double> Capsuled;

    /// \typedef Capsule<float> Capsulef
    /// \brief Capsule with float precision.
    typedef Capsule<float> Capsulef;
    }
  }
}
#include "ignition/math/detail/Capsule.hh"

#endif
