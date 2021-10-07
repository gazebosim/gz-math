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

%module orientedbox
%{
#include <iostream>
#include <ignition/math/OrientedBox.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Material.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/config.hh>
%}

namespace ignition
{
  namespace math
  {
    template<typename T>
    class ignition::math::OrientedBox
    {
      public: OrientedBox() :
        size(ignition::math::Vector3<T>::Zero), pose(ignition::math::Pose3<T>::Zero)
      {
      }

      public: OrientedBox(
        const ignition::math::Vector3<T> &_size, const ignition::math::Pose3<T> &_pose)
          : size(_size.Abs()), pose(_pose)
      {
      }

      public: OrientedBox(const ignition::math::Vector3<T> &_size, const ignition::math::Pose3<T> &_pose,
                  const ignition::math::Material &_mat)
          : size(_size.Abs()), pose(_pose), material(_mat)
      {
      }

      public: explicit OrientedBox(const ignition::math::Vector3<T> &_size)
          : size(_size.Abs()), pose(ignition::math::Pose3<T>::Zero)
      {
      }

      public: explicit OrientedBox(const ignition::math::Vector3<T> &_size,
                                   const ignition::math::Material &_mat)
          : size(_size.Abs()), pose(ignition::math::Pose3<T>::Zero), material(_mat)
      {
      }

      public: OrientedBox(const ignition::math::OrientedBox<T> &_b)
          : size(_b.size), pose(_b.pose), material(_b.material)
      {
      }

      public: virtual ~OrientedBox()
      {
      }

      public: T XLength() const
      {
        return this->size.X();
      }

      public: T YLength() const
      {
        return this->size.Y();
      }

      public: T ZLength() const
      {
        return this->size.Z();
      }

      public: const ignition::math::Vector3<T> &Size() const
      {
        return this->size;
      }

      public: const ignition::math::Pose3<T> &Pose() const
      {
        return this->pose;
      }

      public: void Size(ignition::math::Vector3<T> &_size)
      {
        // Enforce non-negative size
        this->size = _size.Abs();
      }

      public: void Pose(ignition::math::Pose3<T> &_pose)
      {
        this->pose = _pose;
      }

      public: bool operator==(const ignition::math::OrientedBox<T> &_b) const
      {
        return this->size == _b.size && this->pose == _b.pose &&
               this->material == _b.material;
      }

      public: bool operator!=(const ignition::math::OrientedBox<T> &_b) const
      {
        return this->size != _b.size || this->pose != _b.pose ||
               this->material != _b.material;
      }

      public: bool Contains(const ignition::math::Vector3<double> &_p) const
      {
        // Move point to box frame
        auto t = ignition::math::Matrix4<T>(this->pose).Inverse();
        auto p = t *_p;

        return p.X() >= -this->size.X()*0.5 && p.X() <= this->size.X()*0.5 &&
               p.Y() >= -this->size.Y()*0.5 && p.Y() <= this->size.Y()*0.5 &&
               p.Z() >= -this->size.Z()*0.5 && p.Z() <= this->size.Z()*0.5;
      }

      public: const ignition::math::ignition::math::Material &ignition::math::Material() const
      {
        return this->material;
      }

      public: void ignition::math::SetMaterial(const ignition::math::ignition::math::Material &_mat)
      {
        this->material = _mat;
      }

      public: T Volume() const
      {
        return this->size.X() * this->size.Y() * this->size.Z();
      }

      public: T DensityFromMass(const T _mass) const
      {
        if (this->size.Min() <= 0|| _mass <= 0)
          return -1.0;

        return _mass / this->Volume();
      }

      public: bool SetDensityFromMass(const T _mass)
      {
        T newDensity = this->DensityFromMass(_mass);
        if (newDensity > 0)
          this->material.SetDensity(newDensity);
        return newDensity > 0;
      }

      public: bool MassMatrix(MassMatrix3<T> &_massMat) const
      {
        return _massMat.SetFromBox(this->material, this->size);
      }
    };
    %template(OrientedBoxd) OrientedBox<double>;
  }
}
