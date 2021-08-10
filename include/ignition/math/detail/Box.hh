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

#include <queue>
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
struct TriangleFace {
  Triangle3<T> triangle;
  std::size_t indices[3];

  bool operator==(const TriangleFace &_t) const
  {
    std::unordered_set<std::size_t> indices1, indices2;
    indices1.insert(this->indices[0]);
    indices1.insert(this->indices[1]);
    indices1.insert(this->indices[2]);

    indices2.insert(_t.indices[0]);
    indices2.insert(_t.indices[1]);
    indices2.insert(_t.indices[2]);

    return indices1 == indices2;
  }
};
/////////////////////////////////////////////////
template<typename T>
bool isConvexOuterFace(
  const TriangleFace<T> &_face, const std::vector<Vector3<T>> &_vertices)
{
  Plane<T> plane(_face.triangle.Normal(),
    _face.triangle.Normal().Dot(_vertices[_face.indices[0]]));

  std::optional<bool> signPositive = std::nullopt;

  for (auto v: _vertices)
  {
    auto dist = plane.Distance(v);
    if (dist == 0) continue;

    if (dist > 0)
    {
      if (signPositive.has_value())
      {
        if(!signPositive.value()) return false;
      }
      signPositive = true;
    }
    else
    {
      if (signPositive.has_value())
      {
        if(signPositive.value()) return false;
      }
      signPositive = false;
    }
  }

  return true;
}
/////////////////////////////////////////////////
struct Edge
{
  std::size_t a, b;

  Edge(std::size_t _a, std::size_t _b): a(_a), b(_b) {}

  bool operator==(const Edge& other)  const
  {
    return (a == other.a && b == other.b) ||
          (a == other.b && b == other.a);
  }
};
/////////////////////////////////////////////////
struct EdgeHash {
  std::size_t operator()(const Edge &_e) const
  {
    return std::hash<std::size_t>()(_e.a) ^ std::hash<std::size_t>()(_e.b);
  };
};

/////////////////////////////////////////////////
template<typename T>
struct TriangleFaceHash {
  std::size_t operator()(const TriangleFace<T> &_t) const
  {
    return std::hash<std::size_t>()(_t.indices[0])
      ^ std::hash<std::size_t>()(_t.indices[1])
      ^ std::hash<std::size_t>()(_t.indices[2]);
  }
};
/////////////////////////////////////////////////
template<typename T>
bool HasOverlap(
  const TriangleFace<T>& face,
  const std::unordered_set<TriangleFace<T>, TriangleFaceHash<T>> &_faces)
{
  for (auto f: _faces)
  {
    if(f.triangle.Overlaps(face.triangle))
    {
      return true;
    }
  }
  return false;
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

  /// Get the convex hull triangle mesh of the vertices in the shape.
  /// This uses the so called Gift-Wrapping algorithm. It isn't the most
  /// efficient but given the smalll number of points it should be fast enough.
  std::queue<Edge> activeEdges;

  std::unordered_set<TriangleFace<T>, TriangleFaceHash<T>> triangles;
  triangles.insert(TriangleFace<T>{
    Triangle3<T>{
      verticesBelow[verticesBelow.size() - 1],
      verticesBelow[verticesBelow.size() - 2],
      verticesBelow[verticesBelow.size() - 3]},
    {verticesBelow.size() - 1, verticesBelow.size() - 2, verticesBelow.size() - 3}
    });
  activeEdges.emplace(verticesBelow.size() - 1, verticesBelow.size() - 2);
  activeEdges.emplace(verticesBelow.size() - 2, verticesBelow.size() - 3);
  activeEdges.emplace(verticesBelow.size() - 1, verticesBelow.size() - 3);

  std::unordered_set<Edge, EdgeHash> exploredEdges;
  while(!activeEdges.empty())
  {
    auto edge = activeEdges.front();
    activeEdges.pop();
    if(exploredEdges.count(edge) != 0)
      continue;
    exploredEdges.insert(edge);
    for (std::size_t i = 0; i < verticesBelow.size(); ++i)
    {
      if (i == edge.a || i == edge.b) continue;
      // Attempt to create a triangle
      TriangleFace<T> newTriangle{
        Triangle3<T>(
          verticesBelow[edge.a],
          verticesBelow[edge.b],
          verticesBelow[i]),
        {edge.a, edge.b, i}
      };

      std::cout << "Testing edge " << verticesBelow[edge.a]
        << "--" << verticesBelow[edge.b] << std::endl;
      std::cout << "Point: " << verticesBelow[i] << std::endl;

      if(!isConvexOuterFace(newTriangle, verticesBelow)
        || HasOverlap(newTriangle, triangles)) continue;

      std::cout << "Accepted" << std::endl;

      bool triangleIsNew = false;
      Edge newEdge1{edge.a, i};
      if(exploredEdges.count(newEdge1) == 0)
      {
        triangleIsNew = true;
        activeEdges.push(newEdge1);
      }
      Edge newEdge2{edge.b, i};
      if(exploredEdges.count(newEdge2) == 0)
      {
        triangleIsNew = true;
        activeEdges.push(newEdge2);
      }
      if(triangleIsNew)
      {
        triangles.insert(newTriangle);
      }
    }
  }

  // Calculate the volume of the triangles
  // https://n-e-r-v-o-u-s.com/blog/?p=4415
  T volume = 0;
  std::cout << "Triangle Mesh" << std::endl;
  for(auto triangle : triangles)
  {
    std::cout << "\tT: " << std::endl;
    std::cout <<  "\t\t" << verticesBelow[triangle.indices[0]] << std::endl;
    std::cout <<  "\t\t" << verticesBelow[triangle.indices[1]] << std::endl;
    std::cout <<  "\t\t" << verticesBelow[triangle.indices[2]] << std::endl;
    auto crossProduct = verticesBelow[triangle.indices[0]]
        .Cross(verticesBelow[triangle.indices[1]]);
    volume += crossProduct.Dot(verticesBelow[triangle.indices[2]]);
  }

  std::cout << "Triangles " << triangles.size() <<std::endl;

  return abs(volume)/6;
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
