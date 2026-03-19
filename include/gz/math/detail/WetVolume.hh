/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
/// \file WetVolume.hh
/// \brief Helpers for computing the volume and centroid of the portion
/// of a solid of revolution that lies below an arbitrary cutting plane
/// ("wet volume"), used by VolumeBelow / CenterOfVolumeBelow on
/// Cylinder, Capsule, and Ellipsoid.
///
/// Two families of helpers are provided:
///
/// **Circular-segment primitives** -- closed-form area and moment
/// integrals for the region of a circle of radius R that lies on one
/// side of a chord at signed distance p from the center.  The area
/// formula is classical geometry (see Weisstein, "Circular Segment",
/// MathWorld, https://mathworld.wolfram.com/CircularSegment.html).
/// The antiderivatives (F, H, J) are obtained by integration by parts
/// and are used by the Cylinder implementation to evaluate the swept
/// volume and centroid in closed form via Cavalieri's principle.
///
/// **Gauss-Legendre 10-point quadrature** -- numerical integration for
/// integrands that do not admit a closed-form antiderivative.  Nodes and
/// weights are the standard GL-10 values (Abramowitz & Stegun,
/// Table 25.4; DLMF 3.5, https://dlmf.nist.gov/3.5).
#ifndef GZ_MATH_DETAIL_WET_VOLUME_HH_
#define GZ_MATH_DETAIL_WET_VOLUME_HH_

#include <cmath>
#include <gz/math/config.hh>

namespace ignition
{
namespace math
{
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace detail
{
  /// \brief Area of a circular segment for a circle of radius R at
  /// signed distance p from the center.  Valid for |p| <= R.
  /// A(p) = R^2 * arccos(-p/R) + p * sqrt(R^2 - p^2)
  /// \ref https://mathworld.wolfram.com/CircularSegment.html
  template<typename T>
  T circSegArea(T p, T R)
  {
    auto R2 = R * R;
    auto diff = R2 - p * p;
    return R2 * std::acos(-p / R) + p * std::sqrt(diff);
  }

  /// \brief Antiderivative of circSegArea(p, R) w.r.t. p (|p| <= R).
  /// Used by Cylinder::VolumeBelow to integrate the swept segment area
  /// along the cylinder axis in closed form.
  /// F(p) = R^2*p*arccos(-p/R) + R^2*sqrt(R^2-p^2) - (R^2-p^2)^(3/2)/3
  /// F(-R) = 0, F(R) = pi*R^3.
  template<typename T>
  T circSegAreaAntideriv(T p, T R)
  {
    auto R2 = R * R;
    auto diff = R2 - p * p;
    auto sd = std::sqrt(diff);
    return R2 * p * std::acos(-p / R) + R2 * sd - diff * sd / 3;
  }

  /// \brief Antiderivative of p*circSegArea(p, R) w.r.t. p (|p| <= R).
  /// Used by Cylinder::CenterOfVolumeBelow for the z-moment integral.
  /// H(p) = R^2*(4p^2-R^2)/8 * arccos(-p/R) + p*(R^2+2p^2)/8 * sqrt(R^2-p^2)
  /// H(-R) = 0, H(R) = 3*pi*R^4/8.
  template<typename T>
  T circSegPAntideriv(T p, T R)
  {
    auto R2 = R * R;
    auto p2 = p * p;
    auto sd = std::sqrt(R2 - p2);
    return R2 * (4 * p2 - R2) / 8 * std::acos(-p / R)
         + p * (R2 + 2 * p2) / 8 * sd;
  }

  /// \brief Antiderivative of (R^2 - p^2)^(3/2) w.r.t. p (|p| <= R).
  /// Used by Cylinder::CenterOfVolumeBelow for the perpendicular (x,y)
  /// moment integral, derived from the first moment of a circular
  /// segment: m_perp(p) = -(2/3)(R^2 - p^2)^(3/2).
  /// J(p) = p*(5R^2-2p^2)*sqrt(R^2-p^2)/8 + 3R^4*arcsin(p/R)/8
  template<typename T>
  T r2p2_32_Antideriv(T p, T R)
  {
    auto R2 = R * R;
    auto p2 = p * p;
    auto sd = std::sqrt(R2 - p2);
    return p * (5 * R2 - 2 * p2) * sd / 8
         + 3 * R2 * R2 * std::asin(p / R) / 8;
  }

  /// \brief 10-point Gauss-Legendre nodes on [-1, 1].
  /// \ref Abramowitz & Stegun, Table 25.4; DLMF 3.5
  /// (https://dlmf.nist.gov/3.5)
  constexpr double gl10Nodes[10] = {
    -0.97390652851717172, -0.86506336668898451,
    -0.67940956829902441, -0.43339539412924719,
    -0.14887433898163122,  0.14887433898163122,
     0.43339539412924719,  0.67940956829902441,
     0.86506336668898451,  0.97390652851717172
  };

  /// \brief 10-point Gauss-Legendre weights on [-1, 1].
  constexpr double gl10Weights[10] = {
    0.06667134430868814, 0.14945134915058060,
    0.21908636251598204, 0.26926671930999636,
    0.29552422471475287, 0.29552422471475287,
    0.26926671930999636, 0.21908636251598204,
    0.14945134915058060, 0.06667134430868814
  };

  /// \brief Integrate a function f over [a, b] using 10-point
  /// Gauss-Legendre quadrature.  Exact for polynomials up to degree 19.
  template<typename T, typename Func>
  T glIntegrate(Func f, T a, T b)
  {
    auto mid = (a + b) / 2;
    auto halfW = (b - a) / 2;
    T sum = 0;
    for (int i = 0; i < 10; ++i)
      sum += static_cast<T>(gl10Weights[i])
           * f(mid + halfW * static_cast<T>(gl10Nodes[i]));
    return sum * halfW;
  }
}  // namespace detail
}  // namespace IGNITION_MATH_VERSION_NAMESPACE
}  // namespace math
}  // namespace ignition

#endif  // GZ_MATH_DETAIL_WET_VOLUME_HH_
