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
#ifndef GZ_MATH_DETAIL_BOX_HH_
#define GZ_MATH_DETAIL_BOX_HH_

#include "gz/math/Box.hh"
#include "gz/math/Triangle3.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace ignition
{
namespace math
{
//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(T _length, T _width, T _height)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(T _length, T _width, T _height,
    const gz::math::Material &_mat)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(const Vector3<T> &_size)
{
  this->size = _size;
}

//////////////////////////////////////////////////
template<typename T>
Box<T>::Box(const Vector3<T> &_size, const gz::math::Material &_mat)
{
  this->size = _size;
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
math::Vector3<T> Box<T>::Size() const
{
  return this->size;
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetSize(T _length, T _width, T _height)
{
  this->size.X(_length);
  this->size.Y(_width);
  this->size.Z(_height);
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetSize(const math::Vector3<T> &_size)
{
  this->size = _size;
}

//////////////////////////////////////////////////
template<typename T>
const gz::math::Material &Box<T>::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetMaterial(const gz::math::Material &_mat)
{
  this->material = _mat;
}

//////////////////////////////////////////////////
template<typename T>
bool Box<T>::operator==(const Box<T> &_b) const
{
  return this->size == _b.size && this->material == _b.material;
}

//////////////////////////////////////////////////
template<typename T>
bool Box<T>::operator!=(const Box<T> &_b) const
{
  return !(*this == _b);
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::Volume() const
{
  return this->size.X() * this->size.Y() * this->size.Z();
}

//////////////////////////////////////////////////
/// \brief Given a *convex* polygon described by the vertices in a given plane,
/// compute the list of triangles which form this polygon.
/// \param[in] _plane The plane in which the vertices exist.
/// \param[in] _vertices The vertices of the polygon.
/// \return A vector of triangles and their sign, or an empty vector
/// if _vertices in the _plane are less than 3. The sign will be +1 if the
/// triangle is outward facing, -1 otherwise.
/// \note This function relies on exact float comparison
/// (Plane::Side() == NO_SIDE) to classify vertices, which silently
/// drops vertices with any floating-point error and produces wrong
/// triangle decompositions for non-axis-aligned planes.
/// Box::VolumeBelow() and Box::CenterOfVolumeBelow() use robust
/// analytic formulas instead.
/// See https://github.com/gazebosim/gz-math/pull/724
template <typename T>
std::vector<std::pair<Triangle3<T>, T>> TrianglesInPlane(
    const Plane<T> &_plane, IntersectionPoints<T> &_vertices)
{
  std::vector<std::pair<Triangle3<T>, T>> triangles;
  std::vector<Vector3<T>> pointsInPlane;

  Vector3<T> centroid;
  for (const auto &pt : _vertices)
  {
    if (_plane.Side(pt) == Plane<T>::NO_SIDE)
    {
      pointsInPlane.push_back(pt);
      centroid += pt;
    }
  }
  centroid /= T(pointsInPlane.size());

  if (pointsInPlane.size() < 3)
    return {};

  // Choose a basis in the plane of the triangle
  auto axis1 = (pointsInPlane[0] - centroid).Normalize();
  auto axis2 = axis1.Cross(_plane.Normal()).Normalize();

  // Since the polygon is always convex, we can try to create a fan of
  // triangles by sorting the points by their angle in the plane basis.
  std::sort(pointsInPlane.begin(), pointsInPlane.end(),
    [centroid, axis1, axis2] (const Vector3<T> &_a, const Vector3<T> &_b)
    {
      auto aDisplacement = _a - centroid;
      auto bDisplacement = _b - centroid;

      auto aX = axis1.Dot(aDisplacement) / axis1.Length();
      auto aY = axis2.Dot(aDisplacement) / axis2.Length();

      auto bX = axis1.Dot(bDisplacement) / axis1.Length();
      auto bY = axis2.Dot(bDisplacement) / axis2.Length();

      return atan2(aY, aX) < atan2(bY, bX);
    });
  for (std::size_t i = 0; i < pointsInPlane.size(); ++i)
  {
    triangles.emplace_back(
      Triangle3<T>(pointsInPlane[i],
        pointsInPlane[(i + 1) % pointsInPlane.size()], centroid),
      (_plane.Side({0, 0, 0}) == Plane<T>::POSITIVE_SIDE) ? -1 : 1);
  }

  return triangles;
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::VolumeBelow(const Plane<T> &_plane) const
{
  // Analytic inclusion-exclusion formula for the volume of a box below a plane.
  // Reference: Scardovelli & Zaleski (2000), Lehmann & Gekle (2022).
  //
  // The plane equation is: normal . point = offset
  // "Below" means normal . point <= offset (i.e. Distance <= 0).

  const auto &n = _plane.Normal();
  const T d = _plane.Offset();
  const T totalVol = this->Volume();

  // m_i = |n_i|, half_i = size_i / 2
  const T m1 = std::abs(n.X());
  const T m2 = std::abs(n.Y());
  const T m3 = std::abs(n.Z());
  const T h1 = this->size.X() / 2;
  const T h2 = this->size.Y() / 2;
  const T h3 = this->size.Z() / 2;

  // alpha = offset + sum(m_i * half_i)
  // This shifts the coordinate so alpha=0 corresponds to the
  // "most-negative" corner of the box.
  const T alpha = d + m1 * h1 + m2 * h2 + m3 * h3;

  // M_i = m_i * size_i = 2 * m_i * half_i
  const T M1 = m1 * this->size.X();
  const T M2 = m2 * this->size.Y();
  const T M3 = m3 * this->size.Z();
  const T Msum = M1 + M2 + M3;

  // Early returns
  if (alpha <= 0)
    return 0;
  if (alpha >= Msum)
    return totalVol;

  // Count how many M_i are > 0
  int nonzero = (M1 > 0 ? 1 : 0) + (M2 > 0 ? 1 : 0) + (M3 > 0 ? 1 : 0);

  auto cube = [](T x) -> T { return x * x * x; };
  auto clampPos = [](T x) -> T { return x > 0 ? x : 0; };

  if (nonzero == 3)
  {
    // 3D IE formula
    T ie3 = cube(alpha)
      - cube(clampPos(alpha - M1))
      - cube(clampPos(alpha - M2))
      - cube(clampPos(alpha - M3))
      + cube(clampPos(alpha - M1 - M2))
      + cube(clampPos(alpha - M1 - M3))
      + cube(clampPos(alpha - M2 - M3))
      - cube(clampPos(alpha - M1 - M2 - M3));
    return totalVol * ie3 / (6 * M1 * M2 * M3);
  }
  else if (nonzero == 2)
  {
    // 2D IE formula — find the two non-zero M values
    std::array<T, 2> Mv;
    int idx = 0;
    if (M1 > 0) Mv[idx++] = M1;
    if (M2 > 0) Mv[idx++] = M2;
    if (M3 > 0) Mv[idx++] = M3;
    T Ma = Mv[0], Mb = Mv[1];
    auto square = [](T x) -> T { return x * x; };
    T ie2 = square(alpha)
      - square(clampPos(alpha - Ma))
      - square(clampPos(alpha - Mb))
      + square(clampPos(alpha - Ma - Mb));
    return totalVol * ie2 / (2 * Ma * Mb);
  }
  else if (nonzero == 1)
  {
    // 1D case
    T Mk = M1 > 0 ? M1 : (M2 > 0 ? M2 : M3);
    T frac = alpha / Mk;
    return totalVol * std::clamp(frac, T(0), T(1));
  }
  else
  {
    // 0D case: degenerate box
    return alpha >= 0 ? totalVol : 0;
  }
}

/////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
  Box<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  // Analytic first-moment inclusion-exclusion formula.
  // Computes the volumetric centroid of the region of the box below the plane.
  // Uses dimensional reduction: only non-trivial axes (where M_i > 0)
  // participate in the IE sums, with F_k functions matched to the
  // effective dimensionality k.

  const auto &n = _plane.Normal();
  const T d = _plane.Offset();

  const T m1 = std::abs(n.X());
  const T m2 = std::abs(n.Y());
  const T m3 = std::abs(n.Z());
  const T h1 = this->size.X() / 2;
  const T h2 = this->size.Y() / 2;
  const T h3 = this->size.Z() / 2;

  const T alpha = d + m1 * h1 + m2 * h2 + m3 * h3;

  const T M1 = m1 * this->size.X();
  const T M2 = m2 * this->size.Y();
  const T M3 = m3 * this->size.Z();
  const T Msum = M1 + M2 + M3;

  if (alpha <= 0)
    return std::nullopt;

  if (alpha >= Msum)
    return Vector3<T>::Zero;

  auto clampPos = [](T x) -> T { return x > 0 ? x : 0; };

  // F_k(x) = max(0, x)^k / k!
  auto F1 = [&clampPos](T x) -> T { return clampPos(x); };
  auto F2 = [&clampPos](T x) -> T {
    T cx = clampPos(x); return cx * cx / 2;
  };
  auto F3 = [&clampPos](T x) -> T {
    T cx = clampPos(x); return cx * cx * cx / 6;
  };
  auto F4 = [&clampPos](T x) -> T {
    T cx = clampPos(x); return cx * cx * cx * cx / 24;
  };
  auto Fn = [&F1, &F2, &F3, &F4](int _n, T x) -> T {
    switch (_n) {
      case 1: return F1(x);
      case 2: return F2(x);
      case 3: return F3(x);
      case 4: return F4(x);
      default: return T(0);
    }
  };

  const std::array<T, 3> M = {M1, M2, M3};
  const std::array<T, 3> half = {h1, h2, h3};
  const std::array<T, 3> nComp = {n.X(), n.Y(), n.Z()};

  // Identify non-trivial axes (where M_i > 0)
  std::array<int, 3> ntAxes = {};
  int k = 0;
  for (int i = 0; i < 3; ++i)
  {
    if (M[i] > 0)
      ntAxes[k++] = i;
  }

  if (k == 0)
    return alpha >= 0 ? std::optional<Vector3<T>>(Vector3<T>::Zero)
                      : std::nullopt;

  // Compute V_v (volume in v-coordinates, where v_i = M_i * u_i):
  // V_v = sum over subsets S of non-trivial axes: (-1)^|S| F_k(alpha - M_S)
  T Vv = 0;
  for (int mask = 0; mask < (1 << k); ++mask)
  {
    T Msub = 0;
    int bits = 0;
    for (int b = 0; b < k; ++b)
    {
      if (mask & (1 << b))
      {
        Msub += M[ntAxes[b]];
        ++bits;
      }
    }
    T sgn = (bits % 2 == 0) ? T(1) : T(-1);
    Vv += sgn * Fn(k, alpha - Msub);
  }

  if (Vv <= 0)
    return std::nullopt;

  // For each non-trivial axis i, compute the first moment J_i in v-coords:
  // J_i = sum over T subsets of (ntAxes \ {i}):
  //   (-1)^|T| * [F_{k+1}(a) - F_{k+1}(a - M_i) - M_i * F_k(a - M_i)]
  //   where a = alpha - M_T
  // Then: zbar_i = J_i / (M_i * V_v)
  //       centroid_i = sign(n_i) * half_i * (2*zbar_i - 1)

  Vector3<T> result;
  for (int ai = 0; ai < k; ++ai)
  {
    int i = ntAxes[ai];

    // Build list of other non-trivial axes
    std::array<int, 2> others = {};
    int nOthers = 0;
    for (int aj = 0; aj < k; ++aj)
    {
      if (aj != ai)
        others[nOthers++] = ntAxes[aj];
    }

    T Ji = 0;
    for (int mask = 0; mask < (1 << nOthers); ++mask)
    {
      T Msub = 0;
      int bits = 0;
      for (int b = 0; b < nOthers; ++b)
      {
        if (mask & (1 << b))
        {
          Msub += M[others[b]];
          ++bits;
        }
      }
      T sgn = (bits % 2 == 0) ? T(1) : T(-1);
      T a = alpha - Msub;
      Ji += sgn * (Fn(k + 1, a) - Fn(k + 1, a - M[i])
                   - M[i] * Fn(k, a - M[i]));
    }

    T zbar = Ji / (M[i] * Vv);
    T sgn = nComp[i] >= 0 ? T(1) : T(-1);
    result[i] = sgn * half[i] * (2 * zbar - 1);
  }
  // Trivial axes remain 0 (default Vector3 initialization)

  return result;
}

/////////////////////////////////////////////////
template<typename T>
IntersectionPoints<T> Box<T>::VerticesBelow(const Plane<T> &_plane) const
{
  // Get coordinates of all vertice of box
  // TODO(arjo): Cache this for performance
  IntersectionPoints<T> vertices
  {
    Vector3<T>{this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2}
  };

  IntersectionPoints<T> verticesBelow;
  for (const auto &v : vertices)
  {
    if (_plane.Distance(v) <= 0)
    {
      verticesBelow.insert(v);
    }
  }

  return verticesBelow;
}

/////////////////////////////////////////////////
template<typename T>
T Box<T>::DensityFromMass(const T _mass) const
{
  if (this->size.Min() <= 0|| _mass <= 0)
    return -1.0;

  return _mass / this->Volume();
}

/////////////////////////////////////////////////
template<typename T>
bool Box<T>::SetDensityFromMass(const T _mass)
{
  T newDensity = this->DensityFromMass(_mass);
  if (newDensity > 0)
    this->material.SetDensity(newDensity);
  return newDensity > 0;
}

/////////////////////////////////////////////////
template<typename T>
bool Box<T>::MassMatrix(MassMatrix3<T> &_massMat) const
{
  return _massMat.SetFromBox(this->material, this->size);
}


//////////////////////////////////////////////////
template<typename T>
IntersectionPoints<T> Box<T>::Intersections(
        const Plane<T> &_plane) const
{
  IntersectionPoints<T> intersections;
  // These are vertices via which we can describe edges. We only need 4 such
  // vertices
  std::vector<Vector3<T> > vertices
  {
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2}
  };

  // Axes
  std::vector<Vector3<T>> axes
  {
    Vector3<T>{1, 0, 0},
    Vector3<T>{0, 1, 0},
    Vector3<T>{0, 0, 1}
  };

  // There are 12 edges, which are checked along 3 axes from 4 box corner
  // points.
  for (auto &v : vertices)
  {
    for (auto &a : axes)
    {
      auto intersection = _plane.Intersection(v, a);
      if (intersection.has_value() &&
          intersection->X() >= -this->size.X()/2 &&
          intersection->X() <= this->size.X()/2 &&
          intersection->Y() >= -this->size.Y()/2 &&
          intersection->Y() <= this->size.Y()/2 &&
          intersection->Z() >= -this->size.Z()/2 &&
          intersection->Z() <= this->size.Z()/2)
      {
        intersections.insert(intersection.value());
      }
    }
  }

  return intersections;
}

}
}
#endif
