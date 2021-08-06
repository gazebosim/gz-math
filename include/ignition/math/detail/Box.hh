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

#include <unordered_set>
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

/////////////////////////////////////////////////
template<typename T>
std::pair<Line2<T>, Line2<T>> GetIntersectionPairAlongXAxis(
  const std::vector<Vector3<T>> &_points)
{
  for(int i = 0; i < _points.size(); i++)
  {
    for(int j = 0; j < _points.size(); j++)
    {
      if(i == j) continue;

      auto line1 = Line2<T>(_points[i].X(), _points[i].Y(),
        _points[j].X(), _points[j].Y());

      std::unordered_set<int> set{0,1,2,3};
      set.erase(i);
      set.erase(j);

      auto it = set.begin();
      it++;
      auto line2 = Line2<T>(_points[*it].X(), _points[*it].Y(),
         _points[*set.begin()].X(), _points[*set.begin()].Y());

      if(line1.Intersect(line2))
        continue;

      if(!std::isnan(line1.Slope()) && !std::isnan(line2.Slope()))
        return std::make_pair(line1, line2);
    }
  }
}

/////////////////////////////////////////////////
template<typename T>
std::optional<Line2<T>> GetLineOfEntryAlongZPlane(
  const std::vector<Vector3<T>> &_points,
  float _zplane_value,
  Vector3<T> axis = Vector3<T>(0,0,1)
)
{
  Vector3<T> points[2];
  int numPoints = 0;
  for(auto v : _zplane_value)
  {
    if (v.Dot(axis) == _zplane_value)
    {
      points[numPoints] = v;
      numPoints++;
    }

    if (numPoints == 2)
      break;
  }
  if(numPoints != 2)
    return std::nullopt;
  return Line2<T>(points[0], points[1]);
}

/////////////////////////////////////////////////
template<typename>
Vector3<T> axisOfCut(const Box<T>& box, std::vector<Vector3<T>>& intersections)
{
  int numXAxis = 0;
  int numYAxis = 0;
  int numZAxis = 0;

  for(auto point: intersections)
  {
    if(point.X() == -this->size.X()/2 || point.X() == this->size.X()/2)
    {
      numXAxis++;
    }

    if(point.Y() == -this->size.Y()/2 || point.Y() == this->size.Y()/2)
    {
      numYAxis++;
    }

    if(point.Z() == -this->size.Z()/2 || point.Z() == this->size.Z()/2)
    {
      numZAxis++;
    }
  }

  if(numXAxis >= numYAxis && numXAxis >= numZAxis)
  {
    return Vector3<T> (1,0,0);
  }

  if(numYAxis >= numXAxis && numYAxis >= numZAxis)
  {
    return Vector3<T> (1,0,0);
  }

  if(numZAxis >= numYAxis && numZAxis >= numXAxis)
  {
    return Vector3<T> (1,0,0);
  }
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

  std::vector<Vector3<T>> verticesBelow;
  for(auto &v : vertices)
  {
    if(_plane.Distance(v) < 0)
    {
      verticesBelow.push_back(v);
    }
  }

  auto intersectionPoints = GetIntersections(_plane);

  // std::vector<Vector3<T>> polytopeEdges = 

  // Construct a convex hull. Use the Gift-Wrapping method for simplicity
  // Intersection Points will form the first face.
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
