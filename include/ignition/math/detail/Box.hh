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
#ifndef IGNITION_MATH_DETAIL_BOX_HH_
#define IGNITION_MATH_DETAIL_BOX_HH_

#include "ignition/math/Triangle3.hh"

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
    const ignition::math::Material &_mat)
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
Box<T>::Box(const Vector3<T> &_size, const ignition::math::Material &_mat)
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
const ignition::math::Material &Box<T>::Material() const
{
  return this->material;
}

//////////////////////////////////////////////////
template<typename T>
void Box<T>::SetMaterial(const ignition::math::Material &_mat)
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
template <typename T>
std::vector<std::pair<Triangle3<T>, T>> TrianglesInPlane(Plane<T> &_plane,
  std::vector<Vector3<T>> &_vertices)
{
  std::vector<std::pair<Triangle3<T>, T>> triangles;
  std::vector<Vector3<T>> pointsInPlane;
  Vector3<T> centroid;
  for(auto pt: _vertices)
  {
    if(_plane.Side(pt) == Plane<T>::NO_SIDE)
    {
      pointsInPlane.push_back(pt);
      centroid += pt;
    }
  }
  centroid /= pointsInPlane.size();

  if(pointsInPlane.size() < 3)
    return {};

  auto axis1 = (pointsInPlane[0] - centroid).Normalize();
  auto axis2 = axis1.Cross(_plane.Normal()).Normalize();

  std::sort(pointsInPlane.begin(), pointsInPlane.end(),
    [centroid, axis1, axis2] (const Vector3<T> &a, const Vector3<T> &b)
    {
      auto aDisplacement = a - centroid;
      auto bDisplacement = b - centroid;

      auto aX = axis1.Project(aDisplacement);
      auto aY = axis2.Project(aDisplacement);

      auto bX = axis1.Project(bDisplacement);
      auto bY = axis2.Project(bDisplacement);

      return atan2(aY, aX) < atan2(bY, bX);
    });
  for(std::size_t i = 0; i < pointsInPlane.size(); ++i)
  {
    triangles.emplace_back(
      Triangle3<T>(pointsInPlane[i],
        pointsInPlane[(i+1) % pointsInPlane.size()], centroid),
      (_plane.Side({0,0,0}) == Plane<T>::POSITIVE_SIDE) ? -1 : 1);
  }

  return triangles;
}
/////////////////////////////////////////////////
template<typename T>
T Box<T>::VolumeBelow(const Plane<T> &_plane) const
{
  // Get coordinates of all vertice of box
  // TODO: Cache this for performance
  std::vector<Vector3<T> > vertices {
    Vector3<T>{this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2}
  };

  std::vector<Vector3<T> >  verticesBelow;
  for(auto &v : vertices)
  {
    if(_plane.Distance(v) < 0)
    {
      verticesBelow.push_back(v);
    }
  }

  if(verticesBelow.size() == 0)
    return 0;
  //if(verticesBelow.size() == 8)
  //  return Volume();

  auto intersections = GetIntersections(_plane);
  verticesBelow.insert(verticesBelow.end(),
    intersections.begin(), intersections.end());

  // Fit six planes to the vertices in the shape.
  std::vector<std::pair<Triangle3<T>, T>> triangles;

  std::vector<Plane<T>> planes {
    Plane<T>{Vector3<T>{0, 0, 1}, this->Size().Z()/2},
    Plane<T>{Vector3<T>{0, 0, -1}, this->Size().Z()/2},
    Plane<T>{Vector3<T>{1, 0, 0}, this->Size().X()/2},
    Plane<T>{Vector3<T>{-1, 0, 0}, this->Size().X()/2},
    Plane<T>{Vector3<T>{0, 1, 0}, this->Size().Y()/2},
    Plane<T>{Vector3<T>{0, -1, 0}, this->Size().Y()/2},
    _plane
  };

  for(auto &p : planes)
  {
    auto new_triangles = TrianglesInPlane(p, verticesBelow);
    triangles.insert(triangles.end(),
      new_triangles.begin(),
      new_triangles.end());
  }

  // Calculate the volume of the triangles
  // https://n-e-r-v-o-u-s.com/blog/?p=4415
  T volume = 0;
  for(auto triangle : triangles)
  {
    auto crossProduct = (triangle.first[2]).Cross(triangle.first[1]);
    auto meshVolume = std::fabs(crossProduct.Dot(triangle.first[0]));
    volume += triangle.second * meshVolume;
  }

  return std::fabs(volume)/6;
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
std::vector<Vector3<T>> Box<T>::GetIntersections(
        const Plane<T> &_plane) const
{
  std::vector<Vector3<T>> intersections;
  // These are vertice via which we can describe edges. We only need 4 such
  // vertices
  std::vector<Vector3<T> > vertices {
    Vector3<T>{-this->size.X()/2, -this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, this->size.Y()/2, -this->size.Z()/2},
    Vector3<T>{this->size.X()/2, -this->size.Y()/2, this->size.Z()/2},
    Vector3<T>{-this->size.X()/2, this->size.Y()/2, this->size.Z()/2}
  };

  // Axises
  std::vector<Vector3<T>> axes {
    Vector3<T>{1, 0, 0},
    Vector3<T>{0, 1, 0},
    Vector3<T>{0, 0, 1}
  };

  // There are 12 edges. However, we just need 4 points to describe the
  // twelve edges.
  for(auto &v : vertices)
  {
    for(auto &a : axes)
    {
      auto intersection = _plane.Intersect(v, a);
      if(intersection.has_value() &&
        intersection->X() >= -this->size.X()/2 &&
        intersection->X() <= this->size.X()/2 &&
        intersection->Y() >= -this->size.Y()/2 &&
        intersection->Y() <= this->size.Y()/2 &&
        intersection->Z() >= -this->size.Z()/2 &&
        intersection->Z() <= this->size.Z()/2)
      {
        intersections.push_back(intersection.value());
      }
    }
  }

  return intersections;
}

}
}
#endif
