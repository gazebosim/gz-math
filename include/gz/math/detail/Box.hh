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
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <gz/math/config.hh>

namespace ignition
{
namespace math
{
// Inside the versioned namespace, like every other ignition::math detail
// helper: Box<T> lives there, so an unqualified detail:: in its members must
// find this namespace and not a second, unversioned ignition::math::detail.
// MSVC resolves it to the versioned one and fails otherwise.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace detail
{
/// \brief Smallest M_i = |n_i| * size_i that the Box inclusion-exclusion
/// formulas treat as an axis the plane crosses, rather than one it is
/// parallel to. Differencing F(x) - F(x - M_i) cancels catastrophically as
/// M_i shrinks. With one such axis, that leaves the centre of volume with a
/// relative error near eps * (Msum / M_i)^2, while dropping the axis instead
/// errs by about (M_i / Msum)^2, so the cut sits where the two meet,
/// eps^(1/4) * Msum, and both stay near sqrt(eps).
///
/// That balance only holds for one small axis. With two just above the
/// threshold, the rounding errors of the exact formula compound, to a
/// relative error of up to about eps * Msum^4 / (M_i^2 * M_j * M_k): below a
/// plane with normal (2e-5, 1.4e-4, 1) and offset 0.07, the centre of volume
/// of a 1 x 0.15 x 0.15 m box is off by about 3.7e-6 m. Still small, but do
/// not tune the threshold from the one-axis estimate alone.
/// \param[in] _msum Sum of M_i over the three axes.
/// \return The threshold. Zero for integer types, which do not round.
template<typename T>
T BoxNegligibleSpan(T _msum)
{
  return static_cast<T>(_msum *
      std::sqrt(std::sqrt(std::numeric_limits<T>::epsilon())));
}

/// \brief The region of a box below a plane, set up for the Box
/// inclusion-exclusion formulas.
template<typename T>
struct BoxPlaneCut
{
  /// \brief M_i = |n_i| * size_i for each axis.
  std::array<T, 3> span{};

  /// \brief Threshold from BoxNegligibleSpan: axes with M_i at or below it
  /// are dropped.
  T negligible = 0;

  /// \brief Indices of the kept axes, in increasing order. Only the first
  /// keptCount are set.
  std::array<int, 3> keptAxes{};

  /// \brief Number of kept axes. At least 1 whenever 0 < alpha < keptSum,
  /// since with none kept keptSum is 0.
  int keptCount = 0;

  /// \brief Sum of M_i over the kept axes.
  T keptSum = 0;

  /// \brief Plane offset shifted so 0 is the most-negative corner of the
  /// box, less M_i / 2 for each dropped axis. Nothing lies below the plane
  /// when it is at most 0, and all of the box does when it is at least
  /// keptSum.
  T alpha = 0;
};

/// \brief Set up the region of a box below a plane for the Box
/// inclusion-exclusion formulas. An axis whose M_i is negligible is one the
/// plane is parallel to within rounding: drop it as if M_i were zero, moving
/// the plane by the mean of its contribution, M_i / 2, which keeps the volume
/// accurate to second order in M_i. See BoxNegligibleSpan.
/// \param[in] _size Size of the box.
/// \param[in] _plane The plane. Below means normal . point <= offset.
/// \return The cut.
template<typename T>
BoxPlaneCut<T> BoxCutByPlane(const Vector3<T> &_size, const Plane<T> &_plane)
{
  const auto &n = _plane.Normal();

  // m_i = |n_i|, half_i = size_i / 2
  const T m1 = std::abs(n.X());
  const T m2 = std::abs(n.Y());
  const T m3 = std::abs(n.Z());
  const T h1 = _size.X() / 2;
  const T h2 = _size.Y() / 2;
  const T h3 = _size.Z() / 2;

  // alpha = offset + sum(m_i * half_i)
  // This shifts the coordinate so alpha=0 corresponds to the
  // "most-negative" corner of the box.
  const T alpha = _plane.Offset() + m1 * h1 + m2 * h2 + m3 * h3;

  // M_i = m_i * size_i = 2 * m_i * half_i
  BoxPlaneCut<T> cut;
  cut.span = {m1 * _size.X(), m2 * _size.Y(), m3 * _size.Z()};
  const T Msum = cut.span[0] + cut.span[1] + cut.span[2];

  cut.negligible = BoxNegligibleSpan(Msum);
  cut.alpha = alpha;
  for (int i = 0; i < 3; ++i)
  {
    if (cut.span[i] > cut.negligible)
    {
      cut.keptAxes[cut.keptCount++] = i;
      cut.keptSum += cut.span[i];
    }
    else
    {
      cut.alpha -= cut.span[i] / 2;
    }
  }

  // Exactly, alpha >= Msum implies cut.alpha >= keptSum: dropping axes keeps
  // a box wholly below the plane wholly below it. Rounding can break that by
  // an ulp when a dropped M_i is itself a few ulps, so restore it.
  if (alpha >= Msum)
    cut.alpha = std::max(cut.alpha, cut.keptSum);

  return cut;
}
}  // namespace detail
}  // namespace IGNITION_MATH_VERSION_NAMESPACE

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

  const T totalVol = this->Volume();
  const auto cut = detail::BoxCutByPlane(this->size, _plane);

  // Early returns
  if (cut.alpha <= 0)
    return 0;
  if (cut.alpha >= cut.keptSum)
    return totalVol;

  // The IE sums run over the kept axes only. See detail::BoxCutByPlane.
  const T alpha = cut.alpha;
  std::array<T, 3> Mv{};
  for (int j = 0; j < cut.keptCount; ++j)
  {
    Mv[j] = cut.span[cut.keptAxes[j]];
  }

  auto cube = [](T x) -> T { return x * x * x; };
  auto clampPos = [](T x) -> T { return x > 0 ? x : 0; };

  if (cut.keptCount == 3)
  {
    // 3D IE formula
    T ie3 = cube(alpha)
      - cube(clampPos(alpha - Mv[0]))
      - cube(clampPos(alpha - Mv[1]))
      - cube(clampPos(alpha - Mv[2]))
      + cube(clampPos(alpha - Mv[0] - Mv[1]))
      + cube(clampPos(alpha - Mv[0] - Mv[2]))
      + cube(clampPos(alpha - Mv[1] - Mv[2]))
      - cube(clampPos(alpha - Mv[0] - Mv[1] - Mv[2]));
    return totalVol * ie3 / (6 * Mv[0] * Mv[1] * Mv[2]);
  }
  else if (cut.keptCount == 2)
  {
    // 2D IE formula over the two kept axes
    T Ma = Mv[0], Mb = Mv[1];
    auto square = [](T x) -> T { return x * x; };
    T ie2 = square(alpha)
      - square(clampPos(alpha - Ma))
      - square(clampPos(alpha - Mb))
      + square(clampPos(alpha - Ma - Mb));
    return totalVol * ie2 / (2 * Ma * Mb);
  }
  else
  {
    // 1D case. There is no 0D case: past the early returns at least one axis
    // is kept, see detail::BoxPlaneCut::keptCount.
    T frac = alpha / Mv[0];
    return totalVol * std::clamp(frac, T(0), T(1));
  }
}

/////////////////////////////////////////////////
template<typename T>
std::optional<Vector3<T>>
  Box<T>::CenterOfVolumeBelow(const Plane<T> &_plane) const
{
  // Analytic first-moment inclusion-exclusion formula.
  // Computes the volumetric centroid of the region of the box below the plane.
  // Uses dimensional reduction: only non-trivial axes (the kept axes of
  // detail::BoxCutByPlane) participate in the IE sums, with F_k functions
  // matched to the effective dimensionality k.

  const auto cut = detail::BoxCutByPlane(this->size, _plane);
  if (cut.alpha <= 0)
    return std::nullopt;

  if (cut.alpha >= cut.keptSum)
    return Vector3<T>::Zero;

  auto clampPos = [](T x) -> T { return x > 0 ? x : 0; };

  // F_k(x) = max(0, x)^k / k!, with F_0 the unit step
  auto F0 = [](T x) -> T { return x > 0 ? T(1) : T(0); };
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
  auto Fn = [&F0, &F1, &F2, &F3, &F4](int _n, T x) -> T {
    switch (_n) {
      case 0: return F0(x);
      case 1: return F1(x);
      case 2: return F2(x);
      case 3: return F3(x);
      case 4: return F4(x);
      default: return T(0);
    }
  };

  const auto &n = _plane.Normal();
  const std::array<T, 3> &M = cut.span;
  const std::array<T, 3> half = {
    this->size.X() / 2, this->size.Y() / 2, this->size.Z() / 2};
  const std::array<T, 3> nComp = {n.X(), n.Y(), n.Z()};

  // Non-trivial axes. Past the early returns there is at least one, see
  // detail::BoxPlaneCut::keptCount.
  const std::array<int, 3> &ntAxes = cut.keptAxes;
  const int k = cut.keptCount;
  const T alpha = cut.alpha;

  // Compute V_v (volume in v-coordinates, where v_i = M_i * u_i), and its
  // derivative with respect to alpha, which the dropped axes need:
  // V_v = sum over subsets S of non-trivial axes: (-1)^|S| F_k(alpha - M_S)
  T Vv = 0;
  T dVv = 0;
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
    dVv += sgn * Fn(k - 1, alpha - Msub);
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

  // Dropped axes with M_i > 0. Averaging over u_i in [0, 1] puts the centroid
  // at zbar_i = 1/2 - M_i * V_v' / (12 * V_v), with an error of third order
  // in M_i where V_v is smooth: the region leans toward the low side of the
  // plane. Axes with M_i == 0 remain 0 (default Vector3 initialization).
  for (int i = 0; i < 3; ++i)
  {
    if (M[i] > cut.negligible || M[i] <= 0)
      continue;

    T zbar = std::clamp(T(0.5) - M[i] * dVv / (12 * Vv), T(0), T(1));
    T sgn = nComp[i] >= 0 ? T(1) : T(-1);
    result[i] = sgn * half[i] * (2 * zbar - 1);
  }

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
Vector3<T> Box<T>::Centroid() const
{
  return Vector3<T>::Zero;
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
