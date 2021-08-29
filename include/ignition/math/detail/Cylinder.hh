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
#ifndef IGNITION_MATH_DETAIL_CYLINDER_HH_
#define IGNITION_MATH_DETAIL_CYLINDER_HH_
namespace ignition
{
namespace math
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
T Cylinder<T>::Volume() const
{
  return IGN_PI * std::pow(this->radius, 2) *
         this->length;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::VolumeBelow(const Plane<T> &_plane) const
{
  auto length = this->Length();
  auto radius = this->Radius();

  if(_plane.Normal().Dot(Vector3<T>{0, 0, 1}) == T(0))
  {
    // If the plane is parallel to the cylinder's axis
    auto dist = _plane.Distance(Vector3<T>(0,0,0));

    if(abs(dist) >= radius)
    {
      if(dist < 0)
      {
        return Volume();
      }
      else
      {
        return T(0);
      }
    }
    else
    {
      auto volume = CircleSegmentSliceArea(abs(dist)) * this->Length();
      if(dist < 0)
      {
        return volume;
      }
      else
      {
        return this->Volume() - volume;
      }
    }
  }

  //Compute intersection point of plane
  auto theta = atan2(_plane.Normal().Y(), _plane.Normal().X());
  auto x = radius * cos(theta);
  auto y = radius * sin(theta);
  auto point_max = _plane.GetPointOnPlane(x, y);
  x = radius * cos(theta + IGN_PI);
  y = radius * sin(theta + IGN_PI);
  auto point_min = _plane.GetPointOnPlane(x, y);

  //Get case type
  if(point_max.Z() > length/2 && point_min.Z() < -length/2)
  {
    // Plane cuts through both faces
    auto topPoints =
      this->GetCylinderIntersectionsAtZ(_plane, length/2);
    auto bottomPoints =
      this->GetCylinderIntersectionsAtZ(_plane, -length/2);

    auto topChord = topPoints.first - topPoints.second;
  }
  else if(point_max.Z() > length/2 && point_min.Z() >= -length/2)
  {
    // Plane cuts through only top face
    auto topPoints =
      GetCylinderIntersectionsAtZ(_plane, length/2);
    Line3<T> chord(topPoints.first, topPoints.second);
    auto a = chord.Length()/2;
    auto side = _plane.Distance(Vector3<T>{0, 0, length/2});
    //auto b = (side < 0) ?
    //  this->radius - chord.Distance(Vector3<T>{0, 0, length/2}):
    //  this->radius + chord.Distance(Vector3<T>{0, 0, length/2});
  }
  else
  {
    // Plane Cuts through no flat faces.
    auto a = abs(point_max.Z()) + length/2;
    auto b = abs(point_min.Z()) + length/2;
    auto avg_height = (a + b)/2;
    return avg_height * IGN_PI * radius * radius;
  }

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

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::CircleSegmentSliceArea(const T _distance) const
{
  auto r = this->Radius();
  auto theta = (_distance != T(0)) ?
    2 * acos(r/_distance) :
    IGN_PI;
  return r * r * (theta - sin(theta)) / 2;
}

//////////////////////////////////////////////////
template<typename T>
T Cylinder<T>::CylindricalWedgeVolume(
  const T _b, const T _a, const T _h)
{
  auto r = this->Radius();
  auto psi = IGN_PI_2 + atan((_b - r)/_a);
  return _h/(3*_b)*(_a*(3*r*r - _a*_a) + 3*r*r*(_b-r)*psi);
}

//////////////////////////////////////////////////
template<typename T>
std::pair<Vector3<T>, Vector3<T>>
  Cylinder<T>::GetCylinderIntersectionsAtZ(
    const Plane<T> &_plane,
    const T z) const
{
  auto k = (_plane.Offset() - _plane.Normal().Z() * z)
    / this->Radius();
  auto a = _plane.Normal().X();
  auto b = _plane.Normal().Y();

  auto internal = (b - sqrt(a*a + b*b - k*k))/(a+k);
  auto theta1 = 2*(atan(internal));
  auto theta2 = 2*(atan(-internal));

  math::Vector3d intersect1
  {
    this->Radius() * cos(theta1),
    this->Radius() * sin(theta1),
    z
  };

  math::Vector3d intersect2
  {
    this->Radius() * cos(theta2),
    this->Radius() * sin(theta2),
    z
  };

  return {intersect1, intersect2};
}
}
}
#endif
