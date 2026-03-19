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
#ifndef GZ_MATH_DETAIL_CYLINDER_HH_
#define GZ_MATH_DETAIL_CYLINDER_HH_

#include <algorithm>
#include <cmath>
#include <optional>
#include "gz/math/Cylinder.hh"
#include "gz/math/detail/WetVolume.hh"

namespace gz::math
{
//////////////////////////////////////////////////
template<typename T>
Cylinder<T>::Cylinder(const T _length, const T _radius,
    const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
Cylinder<T>::Cylinder(const T _length, const T _radius,
    const Material &_mat, const Quaternion<T> &_rotOffset)
{
  this->length = _length;
  this->radius = _radius;
  this->material = _mat;
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::Radius() const
{
  return this->radius;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetRadius(const T _radius)
{
  this->radius = _radius;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::Length() const
{
  return this->length;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetLength(const T _length)
{
  this->length = _length;
}

//////////////////////////////////////////////////
template<typename T>
Quaternion<T> Cylinder<T>::RotationalOffset() const
{
  return this->rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetRotationalOffset(const Quaternion<T> &_rotOffset)
{
  this->rotOffset = _rotOffset;
}

//////////////////////////////////////////////////
template<typename T>
const Material &Cylinder<T>::Mat() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Cylinder<T>::SetMat(const Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::operator==(const Cylinder &_cylinder) const
{
  return equal(this->radius, _cylinder.Radius()) &&
    equal(this->length, _cylinder.Length()) &&
    this->material == _cylinder.Mat();
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::MassMatrix(MassMatrix3d &_massMat) const
{
  return _massMat.SetFromCylinderZ(
      this->material, this->length,
      this->radius, this->rotOffset);
}

//////////////////////////////////////////////////
template<typename T>
std::optional < MassMatrix3<T> > Cylinder<T>::MassMatrix() const
{
  gz::math::MassMatrix3<T> _massMat;

  if(!_massMat.SetFromCylinderZ(
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
T Cylinder<T>::Volume() const
{
  return GZ_PI * std::pow(this->radius, 2) *
         this->length;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::VolumeBelow(const Plane<T> &_plane) const
{
  auto r = this->radius;
  auto halfLen = this->length / 2;

  if (r <= 0 || this->length <= 0)
    return 0;

  // Transform plane to Z-aligned cylinder frame
  auto localNormal =
    this->rotOffset.RotateVectorReverse(_plane.Normal());
  auto d = _plane.Offset();
  auto nx = localNormal.X();
  auto ny = localNormal.Y();
  auto nz = localNormal.Z();
  auto nxy = std::sqrt(nx * nx + ny * ny);
  auto fullArea = GZ_PI * r * r;

  // Horizontal plane (no radial component)
  if (nxy < 1e-15 * (std::abs(nz) + 1e-30))
  {
    if (std::abs(nz) < 1e-30)
      return (d >= 0) ? this->Volume() : 0;
    auto zCut = d / nz;
    auto h = std::max(static_cast<T>(0),
                      std::min(this->length, zCut + halfLen));
    return fullArea * h;
  }

  // Vertical plane (no axial component)
  if (std::abs(nz) < 1e-15 * nxy)
  {
    auto p = d / nxy;
    T area;
    if (p >= r) area = fullArea;
    else if (p <= -r) area = 0;
    else area = detail::circSegArea(p, r);
    return area * this->length;
  }

  // General case: both nxy > 0 and nz != 0
  // p(z) = (d - nz*z) / nxy at each height z
  auto p1 = (d + nz * halfLen) / nxy;  // p at z = -halfLen
  auto p2 = (d - nz * halfLen) / nxy;  // p at z = +halfLen
  auto pLo = std::min(p1, p2);
  auto pHi = std::max(p1, p2);
  auto dzDp = nxy / std::abs(nz);

  T vol = 0;

  // Full-circle region: p > r
  if (pHi > r)
    vol += fullArea * (pHi - std::max(pLo, r)) * dzDp;

  // Partial region: -r <= p <= r
  auto pLoC = std::max(pLo, -r);
  auto pHiC = std::min(pHi, r);
  if (pLoC < pHiC)
    vol += (detail::circSegAreaAntideriv(pHiC, r)
          - detail::circSegAreaAntideriv(pLoC, r)) * dzDp;

  return vol;
}

//////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
 Cylinder<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  auto r = this->radius;
  auto halfLen = this->length / 2;

  if (r <= 0 || this->length <= 0)
    return std::nullopt;

  auto localNormal =
    this->rotOffset.RotateVectorReverse(_plane.Normal());
  auto d = _plane.Offset();
  auto nx = localNormal.X();
  auto ny = localNormal.Y();
  auto nz = localNormal.Z();
  auto nxy = std::sqrt(nx * nx + ny * ny);
  auto fullArea = GZ_PI * r * r;

  // Horizontal plane
  if (nxy < 1e-15 * (std::abs(nz) + 1e-30))
  {
    if (std::abs(nz) < 1e-30)
    {
      return (d >= 0) ? std::optional(Vector3<T>{0, 0, 0}) : std::nullopt;
    }
    auto zCut = d / nz;
    auto zLo = -halfLen;
    auto h = std::max(static_cast<T>(0),
                      std::min(this->length, zCut + halfLen));
    if (h <= 0) return std::nullopt;
    auto zTop = zLo + h;
    auto cz = (zLo + zTop) / 2;
    return this->rotOffset.RotateVector(Vector3<T>(0, 0, cz));
  }

  // Vertical plane
  if (std::abs(nz) < 1e-15 * nxy)
  {
    auto p = d / nxy;
    T area;
    if (p >= r) area = fullArea;
    else if (p <= -r) area = 0;
    else area = detail::circSegArea(p, r);

    if (area <= 0)
      return std::nullopt;

    // Centroid z = 0 by symmetry.
    // Centroid in 2D normal direction:
    // m_perp = -(2/3)*(R^2-p^2)^(3/2) for |p| < R, else 0
    T cx = 0, cy = 0;
    if (std::abs(p) < r)
    {
      auto diff = r * r - p * p;
      auto mPerp = -(static_cast<T>(2) / 3) * diff * std::sqrt(diff);
      cx = (nx / nxy) * mPerp / area;
      cy = (ny / nxy) * mPerp / area;
    }
    return this->rotOffset.RotateVector(Vector3<T>(cx, cy, 0));
  }

  // General case
  auto p1 = (d + nz * halfLen) / nxy;
  auto p2 = (d - nz * halfLen) / nxy;
  auto pLo = std::min(p1, p2);
  auto pHi = std::max(p1, p2);
  auto dzDp = nxy / std::abs(nz);

  T vol = 0;

  // Volume computation (same as VolumeBelow)
  if (pHi > r)
    vol += fullArea * (pHi - std::max(pLo, r)) * dzDp;
  auto pLoC = std::max(pLo, -r);
  auto pHiC = std::min(pHi, r);
  if (pLoC < pHiC)
    vol += (detail::circSegAreaAntideriv(pHiC, r)
          - detail::circSegAreaAntideriv(pLoC, r)) * dzDp;

  if (vol <= 0)
    return std::nullopt;

  // Z-moment: Mz = (nxy/(|nz|*nz)) * integral of (d-nxy*p)*A(p) dp
  // = (nxy/(|nz|*nz)) * [d*F(p) - nxy*H(p)] evaluated over regions
  T zMomIntegral = 0;

  // Full-circle region: p > r → A = pi*r^2
  // integral of (d - nxy*p) * pi*r^2 dp
  // = pi*r^2 * [d*p - nxy*p^2/2]
  if (pHi > r)
  {
    auto pFullLo = std::max(pLo, r);
    zMomIntegral += fullArea * (d * (pHi - pFullLo)
      - nxy * (pHi * pHi - pFullLo * pFullLo) / 2);
  }

  // Partial region: -r <= p <= r
  if (pLoC < pHiC)
  {
    zMomIntegral +=
      d * (detail::circSegAreaAntideriv(pHiC, r)
         - detail::circSegAreaAntideriv(pLoC, r))
    - nxy * (detail::circSegPAntideriv(pHiC, r)
           - detail::circSegPAntideriv(pLoC, r));
  }

  auto Mz = (nxy / (std::abs(nz) * nz)) * zMomIntegral;

  // XY-moments via perpendicular first moment of circular segment
  // m_perp(p) = -(2/3)*(R^2-p^2)^(3/2) for |p| < R, else 0
  // Mx = -(2*nx)/(3*|nz|) * [J(pHiC) - J(pLoC)]
  // My = -(2*ny)/(3*|nz|) * [J(pHiC) - J(pLoC)]
  T Mx = 0, My = 0;
  if (pLoC < pHiC)
  {
    auto dJ = detail::r2p2_32_Antideriv(pHiC, r)
            - detail::r2p2_32_Antideriv(pLoC, r);
    auto coeff = static_cast<T>(-2) / (3 * std::abs(nz));
    Mx = nx * coeff * dJ;
    My = ny * coeff * dJ;
  }

  auto centroid = Vector3<T>(Mx / vol, My / vol, Mz / vol);
  return this->rotOffset.RotateVector(centroid);
}

//////////////////////////////////////////////////
template<typename T>
bool Cylinder<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::DensityFromMass(const T _mass) const
{
  if (this->radius <= 0 || this->length <=0 || _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}
}  // namespace gz::math
#endif  // GZ_MATH_DETAIL_CYLINDER_HH_
