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

  auto numVerticesBelow = 0;
  for(auto &v : vertices)
  {
    if(_plane.Distance(v) < 0)
    {
      numVerticesBelow++;
    }
  }

  if (numVerticesBelow == 0)
  {
    return 0;
  }
  else if (numVerticesBelow == 1)
  {
    // Get vertex which is below the plance
    Vector3<T> vertex;
    for(auto &v : vertices)
    {
      if(_plane.Distance(v) < 0)
      {
        vertex = v;
      }
    }
    // Compute tetrahedron by getting points of intersection
    auto intersections = GetIntersections(_plane);
    auto area = (intersections[1] - intersections[0]).Cross(intersections[2] - intersections[0]).Length() / 2;
    return area * abs(_plane.Distance(vertex)) / 3;
  }
  else if (numVerticesBelow == 2)
  {
    Plane<T> newPlane(
      _plane.Normal(), _plane.Offset() + (this->size.Z()/2)*_plane.Normal().Z());
    // Compute integral of area under plane bounded by box
    auto intersections = GetIntersections(_plane);
    if(_plane.Normal().Z() != 0)
    {
      //TODO: Determine direction of cut
      // Get integral bounds
      auto [line1, line2] = GetIntersectionPairAlongXAxis(intersections);
      return abs(newPlane.Volume(line1, line2, -this->size.X()/2, this->size.X()/2));
    }
    else
    {
      // Create a a new plane and box with corrected axis.
    }
  }
  else if (numVerticesBelow == 3)
  {

  }
  else if (numVerticesBelow == 4)
  {
    if(_plane.Normal().Z() == 0)
    {
      // Switch Axis

      //Plane<T> newPlane();
    }
    // Compute integral of area under plane bounded by box
    // Setup bounds
    Plane<T> newPlane(
      _plane.Normal(), _plane.Offset() + (this->size.Z()/2)*_plane.Normal().Z());
    Line2<T> line1(-this->size.X()/2, -this->size.Y()/2,
                    this->size.X()/2, -this->size.Y()/2);

    Line2<T> line2(-this->size.X()/2, this->size.Y()/2,
                    this->size.X()/2, this->size.Y()/2);

    return newPlane.Volume(line2, line1, -this->size.X()/2, this->size.X()/2);
  }
  else if (numVerticesBelow == 5)
  {
    // Compute ??
  }
  else if (numVerticesBelow == 6)
  {
    // Compute
  }
  else if (numVerticesBelow == 7)
  {
    // Get vertex which is below the plance
    Vector3<T> vertex;
    for(auto &v : vertices)
    {
      if(_plane.Distance(v) > 0)
      {
        vertex = v;
      }
    }
    // Compute tetrahedron by getting points of intersection
    auto intersections = this->GetIntersections(_plane);
    auto area = 0.5 * (intersections[1] - intersections[0]).Cross(intersections[2] - intersections[0]).Length();
    return this->Volume() - area * abs(_plane.Distance(vertex)) / 3;
  }
  else
  {
    return this->Volume();
  }
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
