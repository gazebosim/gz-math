/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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
#ifndef GZ_MATH_DETAIL_CONE_HH_
#define GZ_MATH_DETAIL_CONE_HH_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include "gz/math/Cone.hh"
#include "gz/math/detail/WetVolume.hh"

namespace gz::math
{
//////////////////////////////////////////////////
template<typename T>
Cone<T>::Cone(const T _length, const T _radius,
    const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
Cone<T>::Cone(const T _length, const T _radius,
    const Material &_mat, const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->material = _mat;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
T Cone<T>::Radius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
template<typename T>
void Cone<T>::SetRadius(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
T Cone<T>::Length() const
{
  return this->length;
}

//////////////////////////////////////////////////
template<typename T>
void Cone<T>::SetLength(const T _length)
{
  this->length = _length;
}

//////////////////////////////////////////////////
template<typename T>
Quaternion<T> Cone<T>::RotationalOffset() const
{
  return this->rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
void Cone<T>::SetRotationalOffset(const Quaternion<T> &_rotOffset)
{
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
const Material &Cone<T>::Mat() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Cone<T>::SetMat(const Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Cone<T>::operator==(const Cone &_cone) const
{
  return equal(this->radius, _cone.Radius()) &&
    equal(this->length, _cone.Length()) &&
    this->material == _cone.Mat();
}

//////////////////////////////////////////////////
template<typename T>
bool Cone<T>::MassMatrix(MassMatrix3d &_massMat) const
{
  return _massMat.SetFromConeZ(
      this->material, this->length,
      this->radius, this->rotOffset);
}

//////////////////////////////////////////////////
template<typename T>
std::optional < MassMatrix3<T> > Cone<T>::MassMatrix() const
{
  gz::math::MassMatrix3<T> _massMat;

  if(!_massMat.SetFromConeZ(
      this->material, this->length,
      this->radius, this->rotOffset))
  {
    return std::nullopt;
  }
  else
  {
    return std::make_optional(_massMat);
  }
}

//////////////////////////////////////////////////
template<typename T>
T Cone<T>::Volume() const
{
  return GZ_PI * std::pow(this->radius, 2) *
         this->length / 3.0;
}

//////////////////////////////////////////////////
template<typename T>
T Cone<T>::VolumeBelow(const Plane<T> &_plane) const
{
  auto R = this->radius;
  auto L = this->length;
  auto halfLen = L / 2;

  if (R <= 0 || L <= 0)
    return 0;

  // Transform plane to Z-aligned cone frame
  auto localNormal =
    this->rotOffset.RotateVectorReverse(_plane.Normal());
  auto d = _plane.Offset();
  auto nx = localNormal.X();
  auto ny = localNormal.Y();
  auto nz = localNormal.Z();
  auto nxy = std::sqrt(nx * nx + ny * ny);

  // Cone geometry: base at z = -L/2 (radius R), apex at z = +L/2 (radius 0)
  // r(z) = R * (L/2 - z) / L
  // "Below" the plane means n . x <= d.

  // Horizontal plane (nxy ≈ 0)
  if (nxy < 1e-15 * (std::abs(nz) + 1e-30))
  {
    if (std::abs(nz) < 1e-30)
      return (d >= 0) ? this->Volume() : 0;
    auto zCut = std::max(-halfLen, std::min(halfLen, d / nz));
    // V = (piR^2 / 3L^2) * (L^3 - (L/2 - zCut)^3)
    auto uCut = halfLen - zCut;
    return GZ_PI * R * R / (3 * L * L) * (L * L * L - uCut * uCut * uCut);
  }

  // Find kink points where p(z) = ±r(z)
  // p(z) = (d - nz*z) / nxy, r(z) = R*(L/2 - z)/L
  // p = r: z_kink1 = (nxy*R/2 - d) / (nxy*R/L - nz)  [if denom != 0]
  // p = -r: z_kink2 = (nxy*R/2 + d) / (nxy*R/L + nz)  [if denom != 0]
  std::array<T, 4> bounds = {-halfLen, halfLen, halfLen, halfLen};
  int nBounds = 2;

  auto denom1 = nxy * R / L - nz;
  if (std::abs(denom1) > 1e-15)
  {
    auto zk = (nxy * R / 2 - d) / denom1;
    if (zk > -halfLen && zk < halfLen)
      bounds[nBounds++] = zk;
  }
  auto denom2 = nxy * R / L + nz;
  if (std::abs(denom2) > 1e-15)
  {
    auto zk = (nxy * R / 2 + d) / denom2;
    if (zk > -halfLen && zk < halfLen)
      bounds[nBounds++] = zk;
  }
  std::sort(bounds.begin(), bounds.begin() + nBounds);

  // Integrate using GL quadrature on each sub-interval
  auto integrand = [R, L, halfLen, nxy, nz, d](T z) -> T {
    auto rz = R * (halfLen - z) / L;
    if (rz <= 0) return 0;
    auto p = (d - nz * z) / nxy;
    if (p >= rz) return GZ_PI * rz * rz;
    if (p <= -rz) return static_cast<T>(0);
    return detail::circSegArea(p, rz);
  };

  T vol = 0;
  for (int i = 0; i < nBounds - 1; ++i)
    vol += detail::glIntegrate(integrand, bounds[i], bounds[i + 1]);

  return std::max(static_cast<T>(0), vol);
}

//////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
 Cone<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  auto R = this->radius;
  auto L = this->length;
  auto halfLen = L / 2;

  if (R <= 0 || L <= 0)
    return std::nullopt;

  auto localNormal =
    this->rotOffset.RotateVectorReverse(_plane.Normal());
  auto d = _plane.Offset();
  auto nx = localNormal.X();
  auto ny = localNormal.Y();
  auto nz = localNormal.Z();
  auto nxy = std::sqrt(nx * nx + ny * ny);

  // Horizontal plane
  if (nxy < 1e-15 * (std::abs(nz) + 1e-30))
  {
    if (std::abs(nz) < 1e-30)
      return (d >= 0) ? std::optional(Vector3<T>{0, 0, 0}) : std::nullopt;

    auto zCut = std::max(-halfLen, std::min(halfLen, d / nz));
    auto uCut = halfLen - zCut;
    auto vol = GZ_PI * R * R / (3 * L * L)
             * (L * L * L - uCut * uCut * uCut);
    if (vol <= 0) return std::nullopt;

    // Mz = integral of z * pi * r(z)^2 dz from -L/2 to zCut
    // = piR^2/L^2 * integral of z*(L/2-z)^2 dz
    // With u = L/2-z: z = L/2-u, integral = integral of (L/2-u)*u^2 (-du)
    // = integral_uCut^L (L/2-u)*u^2 du = L/2*[u^3/3] - [u^4/4]
    auto Mz = GZ_PI * R * R / (L * L) * (
      halfLen * (L * L * L - uCut * uCut * uCut) / 3
      - (L * L * L * L - uCut * uCut * uCut * uCut) / 4);
    return this->rotOffset.RotateVector(Vector3<T>(0, 0, Mz / vol));
  }

  // Find kink points (same as VolumeBelow)
  std::array<T, 4> bounds = {-halfLen, halfLen, halfLen, halfLen};
  int nBounds = 2;
  auto denom1 = nxy * R / L - nz;
  if (std::abs(denom1) > 1e-15)
  {
    auto zk = (nxy * R / 2 - d) / denom1;
    if (zk > -halfLen && zk < halfLen)
      bounds[nBounds++] = zk;
  }
  auto denom2 = nxy * R / L + nz;
  if (std::abs(denom2) > 1e-15)
  {
    auto zk = (nxy * R / 2 + d) / denom2;
    if (zk > -halfLen && zk < halfLen)
      bounds[nBounds++] = zk;
  }
  std::sort(bounds.begin(), bounds.begin() + nBounds);

  // 2D normal direction for radial moments
  auto nhat_x = (nxy > 1e-30) ? nx / nxy : static_cast<T>(0);
  auto nhat_y = (nxy > 1e-30) ? ny / nxy : static_cast<T>(0);

  // Segment area at height z
  auto segArea = [R, L, halfLen, nxy, nz, d](T z) -> T {
    auto rz = R * (halfLen - z) / L;
    if (rz <= 0) return 0;
    auto p = (d - nz * z) / nxy;
    if (p >= rz) return GZ_PI * rz * rz;
    if (p <= -rz) return static_cast<T>(0);
    return detail::circSegArea(p, rz);
  };

  // z * segment area
  auto zTimesArea = [&segArea](T z) -> T {
    return z * segArea(z);
  };

  // Perpendicular first moment of circular segment: -(2/3)(r^2 - p^2)^(3/2)
  auto perpMoment = [R, L, halfLen, nxy, nz, d](T z) -> T {
    auto rz = R * (halfLen - z) / L;
    if (rz <= 0) return 0;
    auto p = (d - nz * z) / nxy;
    if (std::abs(p) >= rz) return static_cast<T>(0);
    auto diff = rz * rz - p * p;
    return -(static_cast<T>(2) / 3) * diff * std::sqrt(diff);
  };

  T vol = 0, Mz = 0, Mperp = 0;
  for (int i = 0; i < nBounds - 1; ++i)
  {
    vol += detail::glIntegrate(segArea, bounds[i], bounds[i + 1]);
    Mz += detail::glIntegrate(zTimesArea, bounds[i], bounds[i + 1]);
    Mperp += detail::glIntegrate(perpMoment, bounds[i], bounds[i + 1]);
  }

  if (vol <= 0)
    return std::nullopt;

  auto cx = nhat_x * Mperp / vol;
  auto cy = nhat_y * Mperp / vol;
  auto cz = Mz / vol;

  return this->rotOffset.RotateVector(Vector3<T>(cx, cy, cz));
}

//////////////////////////////////////////////////
template<typename T>
bool Cone<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

//////////////////////////////////////////////////
template<typename T>
T Cone<T>::DensityFromMass(const T _mass) const
{
  if (this->radius <= 0 || this->length <=0 || _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}

}  // namespace gz::math
#endif  // GZ_MATH_DETAIL_CONE_HH_
