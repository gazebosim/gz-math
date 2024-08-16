/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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

#include "gz/math/Angle.hh"
#include "gz/math/CoordinateVector3.hh"
#include "gz/utils/ImplPtr.hh"

#include <ostream>
#include <variant>

/// \brief Private data for the CoordinateVector3 class.
class gz::math::CoordinateVector3::Implementation
{
  /// \brief The metric x coordinate or spherical latitude.
  public: std::variant<double, math::Angle> x_or_lat;
  /// \brief The metric y coordinate or spherical longitude.
  public: std::variant<double, math::Angle> y_or_lon;
  /// \brief The z coordinate (always metric).
  public: double z;
};

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
CoordinateVector3::CoordinateVector3()
: dataPtr(gz::utils::MakeUniqueImpl<Implementation>(0.0, 0.0, 0.0))
{

}

/////////////////////////////////////////////////
CoordinateVector3::CoordinateVector3(const CoordinateVector3 &_other)
: dataPtr(_other.IsMetric() ?
  gz::utils::MakeUniqueImpl<Implementation>(
    _other.X(), _other.Y(), _other.Z()) :
  gz::utils::MakeUniqueImpl<Implementation>(
    _other.Lat(), _other.Lon(), _other.Z()))
{
}

CoordinateVector3& CoordinateVector3::operator=(const CoordinateVector3 &_other)
{
  if (_other.IsMetric())
    this->SetMetric(_other.X(), _other.Y(), _other.Z());
  else
    this->SetSpherical(_other.Lat(), _other.Lon(), _other.Z());
  return *this;
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::Metric(
    const double _x, const double _y, const double _z)
{
  CoordinateVector3 res;
  res.SetMetric(_x, _y, _z);
  return res;
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::Metric(const Vector3d &_v)
{
  return Metric(_v.X(), _v.Y(), _v.Z());
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::Spherical(
    const Angle &_lat, const Angle &_lon, const double _z)
{
  CoordinateVector3 res;
  res.SetSpherical(_lat, _lon, _z);
  return res;
}

/////////////////////////////////////////////////
bool CoordinateVector3::IsMetric() const
{
  return std::holds_alternative<double>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
bool CoordinateVector3::IsSpherical() const
{
  return std::holds_alternative<math::Angle>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
void CoordinateVector3::SetMetric(
  const double _x, const double _y, const double _z)
{
  this->dataPtr->x_or_lat = _x;
  this->dataPtr->y_or_lon = _y;
  this->dataPtr->z = _z;
}

/////////////////////////////////////////////////
void CoordinateVector3::SetMetric(const Vector3d &_v)
{
  this->SetMetric(_v.X(), _v.Y(), _v.Z());
}

/////////////////////////////////////////////////
void CoordinateVector3::SetSpherical(
  const Angle &_lat, const Angle &_lon, double _z)
{
  this->dataPtr->x_or_lat = _lat;
  this->dataPtr->y_or_lon = _lon;
  this->dataPtr->z = _z;
}

/////////////////////////////////////////////////
math::Vector3d CoordinateVector3::AsMetricVector() const
{
  if (!this->IsMetric())
    throw std::bad_variant_access();
  return {this->X(), this->Y(), this->Z()};
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::operator+(
  const CoordinateVector3 &_v) const
{
  if (this->IsMetric() != _v.IsMetric())
    throw std::bad_variant_access();

  if (this->IsMetric())
    return CoordinateVector3::Metric(
      this->X() + _v.X(), this->Y() + _v.Y(), this->Z() + _v.Z());
  else
    return CoordinateVector3::Spherical(
      this->Lat() + _v.Lat(), this->Lon() + _v.Lon(), this->Z() + _v.Z());
}

/////////////////////////////////////////////////
const CoordinateVector3& CoordinateVector3::operator+=(
  const CoordinateVector3 &_v)
{
  if (this->IsMetric() != _v.IsMetric())
    throw std::bad_variant_access();

  if (this->IsMetric())
  {
    this->X() += _v.X();
    this->Y() += _v.Y();
  }
  else
  {
    this->Lat() += _v.Lat();
    this->Lon() += _v.Lon();
  }
  this->Z() += _v.Z();

  return *this;
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::operator-() const
{
  if (this->IsMetric())
    return Metric(-this->X(), -this->Y(), -this->Z());
  else
    return Spherical({-this->Lat().Radian()},
                     {-this->Lon().Radian()},
                     -this->Z());
}

/////////////////////////////////////////////////
CoordinateVector3 CoordinateVector3::operator-(
  const CoordinateVector3 &_pt) const
{
  if (this->IsMetric() != _pt.IsMetric())
    throw std::bad_variant_access();

  if (this->IsMetric())
    return Metric(
      this->X() - _pt.X(), this->Y() - _pt.Y(), this->Z() - _pt.Z());
  else
    return Spherical(this->Lat() - _pt.Lat(),
                     this->Lon() - _pt.Lon(),
                     this->Z() - _pt.Z());
}

/////////////////////////////////////////////////
const CoordinateVector3& CoordinateVector3::operator-=(
  const CoordinateVector3 &_pt)
{
  if (this->IsMetric())
  {
    this->X() -= _pt.X();
    this->Y() -= _pt.Y();
  }
  else
  {
    this->Lat() -= _pt.Lat();
    this->Lon() -= _pt.Lon();
  }
  this->Z() -= _pt.Z();

  return *this;
}

/////////////////////////////////////////////////
bool CoordinateVector3::Equal(
  const CoordinateVector3 &_v, const double &_tol, const Angle &_ang_tol) const
{
  if (this->IsMetric() != _v.IsMetric())
    return false;

  if (!equal(this->Z(), _v.Z(), _tol))
    return false;

  if (this->IsMetric())
  {
    return equal(this->X(), _v.X(), _tol) && equal(this->Y(), _v.Y(), _tol);
  }
  else
  {
    return this->Lat().ShortestDistance(_v.Lat()).Abs() < _ang_tol &&
           this->Lon().ShortestDistance(_v.Lon()).Abs() < _ang_tol;
  }
}

/////////////////////////////////////////////////
bool CoordinateVector3::operator==(const CoordinateVector3 &_v) const
{
  return this->Equal(_v, 1e-3, 1e-3);
}

/////////////////////////////////////////////////
bool CoordinateVector3::operator!=(const CoordinateVector3 &_v) const
{
  return !(*this == _v);
}

/////////////////////////////////////////////////
bool CoordinateVector3::IsFinite() const
{
  if (!std::isfinite(this->Z()))
    return false;
  if (this->IsMetric())
    return std::isfinite(this->X()) && std::isfinite(this->Y());
  else
    return std::isfinite(this->Lat().Radian()) &&
           std::isfinite(this->Lon().Radian());
}

/////////////////////////////////////////////////
bool CoordinateVector3::Equal(const CoordinateVector3 &_v) const
{
  return *this == _v;
}

/////////////////////////////////////////////////
double CoordinateVector3::X() const
{
  return std::get<double>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
const math::Angle& CoordinateVector3::Lat() const
{
  return std::get<math::Angle>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
double CoordinateVector3::Y() const
{
  return std::get<double>(this->dataPtr->y_or_lon);
}

/////////////////////////////////////////////////
const math::Angle& CoordinateVector3::Lon() const
{
  return std::get<math::Angle>(this->dataPtr->y_or_lon);
}

/////////////////////////////////////////////////
double CoordinateVector3::Z() const
{
  return this->dataPtr->z;
}

/////////////////////////////////////////////////
double& CoordinateVector3::X()
{
  return std::get<double>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
math::Angle& CoordinateVector3::Lat()
{
  return std::get<math::Angle>(this->dataPtr->x_or_lat);
}

/////////////////////////////////////////////////
double& CoordinateVector3::Y()
{
  return std::get<double>(this->dataPtr->y_or_lon);
}

/////////////////////////////////////////////////
math::Angle& CoordinateVector3::Lon()
{
  return std::get<math::Angle>(this->dataPtr->y_or_lon);
}

/////////////////////////////////////////////////
double& CoordinateVector3::Z()
{
  return this->dataPtr->z;
}

/////////////////////////////////////////////////
void CoordinateVector3::X(const double &_v)
{
  if (!this->IsMetric())
    throw std::bad_variant_access();
  this->dataPtr->x_or_lat = _v;
}

/////////////////////////////////////////////////
void CoordinateVector3::Lat(const Angle &_v)
{
  if (!this->IsSpherical())
    throw std::bad_variant_access();
  this->dataPtr->x_or_lat = _v;
}

/////////////////////////////////////////////////
void CoordinateVector3::Y(const double &_v)
{
  if (!this->IsMetric())
    throw std::bad_variant_access();
  this->dataPtr->y_or_lon = _v;
}

/////////////////////////////////////////////////
void CoordinateVector3::Lon(const Angle &_v)
{
  if (!this->IsSpherical())
    throw std::bad_variant_access();
  this->dataPtr->y_or_lon = _v;
}

/////////////////////////////////////////////////
void CoordinateVector3::Z(const double &_v)
{
  this->dataPtr->z = _v;
}
