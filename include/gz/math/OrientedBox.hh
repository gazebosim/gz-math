/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_ORIENTEDBOX_HH_
#define IGNITION_MATH_ORIENTEDBOX_HH_

#include <iostream>
#include <ignition/math/Helpers.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    /// \brief Mathematical representation of a box which can be arbitrarily
    /// positioned and rotated.
    template<typename T>
    class OrientedBox
    {
      /// \brief Default constructor
      public: OrientedBox() : size(Vector3<T>::Zero), pose(Pose3<T>::Zero)
      {
      }

      /// \brief Constructor which takes size and pose.
      /// \param[in] _size Box size, in its own coordinate frame. Its absolute
      /// value will be taken, so the size is non-negative.
      /// \param[in] _pose Box pose.
      public: OrientedBox(const Vector3<T> &_size, const Pose3<T> &_pose)
          : size(_size.Abs()), pose(_pose)
      {
      }

      /// \brief Constructor which takes size, pose, and material.
      /// \param[in] _size Box size, in its own coordinate frame. Its absolute
      /// value will be taken, so the size is non-negative.
      /// \param[in] _pose Box pose.
      /// \param[in] _mat Material property for the box.
      public: OrientedBox(const Vector3<T> &_size, const Pose3<T> &_pose,
                  const Material &_mat)
          : size(_size.Abs()), pose(_pose), material(_mat)
      {
      }

      /// \brief Constructor which takes only the size.
      /// \param[in] _size Box size, in its own coordinate frame. Its absolute
      /// value will be taken, so the size is non-negative.
      public: explicit OrientedBox(const Vector3<T> &_size)
          : size(_size.Abs()), pose(Pose3<T>::Zero)
      {
      }

      /// \brief Constructor which takes only the size.
      /// \param[in] _size Box size, in its own coordinate frame. Its absolute
      /// value will be taken, so the size is non-negative.
      /// \param[in] _mat Material property for the box.
      public: explicit OrientedBox(const Vector3<T> &_size,
                                   const Material &_mat)
          : size(_size.Abs()), pose(Pose3<T>::Zero), material(_mat)
      {
      }

      /// \brief Copy constructor.
      /// \param[in] _b OrientedBox to copy.
      public: OrientedBox(const OrientedBox<T> &_b)
          : size(_b.size), pose(_b.pose), material(_b.material)
      {
      }

      /// \brief Destructor
      public: virtual ~OrientedBox()
      {
      }

      /// \brief Get the length along the x dimension
      /// \return Value of the length in the x dimension
      public: T XLength() const
      {
        return this->size.X();
      }

      /// \brief Get the length along the y dimension
      /// \return Value of the length in the y dimension
      public: T YLength() const
      {
        return this->size.Y();
      }

      /// \brief Get the length along the z dimension
      /// \return Value of the length in the z dimension
      public: T ZLength() const
      {
        return this->size.Z();
      }

      /// \brief Get the size of the box
      /// \return Size of the box
      public: const Vector3<T> &Size() const
      {
        return this->size;
      }

      /// \brief Get the box pose, which is the pose of its center.
      /// \return The pose of the box.
      public: const Pose3<T> &Pose() const
      {
        return this->pose;
      }

      /// \brief Set the box size.
      /// \param[in] _size Box size, in its own coordinate frame. Its absolute
      /// value will be taken, so the size is non-negative.
      public: void Size(Vector3<T> &_size)
      {
        // Enforce non-negative size
        this->size = _size.Abs();
      }

      /// \brief Set the box pose.
      /// \param[in] _pose Box pose.
      public: void Pose(Pose3<T> &_pose)
      {
        this->pose = _pose;
      }

      /// \brief Assignment operator. Set this box to the parameter
      /// \param[in]  _b OrientedBox to copy
      /// \return The new box.
      public: OrientedBox &operator=(const OrientedBox<T> &_b)
      {
        this->size = _b.size;
        this->pose = _b.pose;
        this->material = _b.material;
        return *this;
      }

      /// \brief Equality test operator
      /// \param[in] _b OrientedBox to test
      /// \return True if equal
      public: bool operator==(const OrientedBox<T> &_b) const
      {
        return this->size == _b.size && this->pose == _b.pose &&
               this->material == _b.material;
      }

      /// \brief Inequality test operator
      /// \param[in] _b OrientedBox to test
      /// \return True if not equal
      public: bool operator!=(const OrientedBox<T> &_b) const
      {
        return this->size != _b.size || this->pose != _b.pose ||
               this->material != _b.material;
      }

      /// \brief Output operator
      /// \param[in] _out Output stream
      /// \param[in] _b OrientedBox to output to the stream
      /// \return The stream
      public: friend std::ostream &operator<<(std::ostream &_out,
                                              const OrientedBox<T> &_b)
      {
        _out << "Size[" << _b.Size() << "] Pose[" << _b.Pose() << "] "
          << "Material[" << _b.Material().Name() << "]";
        return _out;
      }

      /// \brief Check if a point lies inside the box.
      /// \param[in] _p Point to check.
      /// \return True if the point is inside the box.
      public: bool Contains(const Vector3d &_p) const
      {
        // Move point to box frame
        auto t = Matrix4<T>(this->pose).Inverse();
        auto p = t *_p;

        return p.X() >= -this->size.X()*0.5 && p.X() <= this->size.X()*0.5 &&
               p.Y() >= -this->size.Y()*0.5 && p.Y() <= this->size.Y()*0.5 &&
               p.Z() >= -this->size.Z()*0.5 && p.Z() <= this->size.Z()*0.5;
      }

      /// \brief Get the material associated with this box.
      /// \return The material assigned to this box.
      public: const ignition::math::Material &Material() const
      {
        return this->material;
      }

      /// \brief Set the material associated with this box.
      /// \param[in] _mat The material assigned to this box.
      public: void SetMaterial(const ignition::math::Material &_mat)
      {
        this->material = _mat;
      }

      /// \brief Get the volume of the box in m^3.
      /// \return Volume of the box in m^3.
      public: T Volume() const
      {
        return this->size.X() * this->size.Y() * this->size.Z();
      }

      /// \brief Compute the box's density given a mass value. The
      /// box is assumed to be solid with uniform density. This
      /// function requires the box's size to be set to
      /// values greater than zero. The Material of the box is ignored.
      /// \param[in] _mass Mass of the box, in kg. This value should be
      /// greater than zero.
      /// \return Density of the box in kg/m^3. A negative value is
      /// returned if the size or _mass is <= 0.
      public: T DensityFromMass(const T _mass) const
      {
        if (this->size.Min() <= 0|| _mass <= 0)
          return -1.0;

        return _mass / this->Volume();
      }

      /// \brief Set the density of this box based on a mass value.
      /// Density is computed using
      /// double DensityFromMass(const double _mass) const. The
      /// box is assumed to be solid with uniform density. This
      /// function requires the box's size to be set to
      /// values greater than zero. The existing Material density value is
      /// overwritten only if the return value from this true.
      /// \param[in] _mass Mass of the box, in kg. This value should be
      /// greater than zero.
      /// \return True if the density was set. False is returned if the
      /// box's size or the _mass value are <= 0.
      /// \sa double DensityFromMass(const double _mass) const
      public: bool SetDensityFromMass(const T _mass)
      {
        T newDensity = this->DensityFromMass(_mass);
        if (newDensity > 0)
          this->material.SetDensity(newDensity);
        return newDensity > 0;
      }

      /// \brief Get the mass matrix for this box. This function
      /// is only meaningful if the box's size and material
      /// have been set.
      /// \param[out] _massMat The computed mass matrix will be stored here.
      /// \return False if computation of the mass matrix failed, which
      /// could be due to an invalid size (<=0) or density (<=0).
      public: bool MassMatrix(MassMatrix3<T> &_massMat) const
      {
        return _massMat.SetFromBox(this->material, this->size);
      }

      /// \brief The size of the box in its local frame.
      private: Vector3<T> size;

      /// \brief The pose of the center of the box.
      private: Pose3<T> pose;

      /// \brief The box's material.
      private: ignition::math::Material material;
    };

    typedef OrientedBox<int> OrientedBoxi;
    typedef OrientedBox<double> OrientedBoxd;
    typedef OrientedBox<float> OrientedBoxf;
    }
  }
}
#endif
