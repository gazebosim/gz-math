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
#include <gtest/gtest.h>
#include <cmath>

#include "gz/math/Box.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(BoxTest, Constructor)
{
  // Default constructor
  {
    math::Boxd box;
    EXPECT_EQ(math::Vector3d::Zero, box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2;
    EXPECT_EQ(box, box2);
  }

  // Individual dimension constructor
  {
    math::Boxd box(1.0, 2.0, 3.0);
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 3.0), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(1.0, 2.0, 3.0);
    EXPECT_EQ(box, box2);
  }

  // Vector dimension constructor
  {
    math::Boxd box(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(math::Vector3d(1.3, 2.5, 4.6), box.Size());
    EXPECT_EQ(math::Material(), box.Material());

    math::Boxd box2(math::Vector3d(1.3, 2.5, 4.6));
    EXPECT_EQ(box, box2);
  }

  // Dimension and mat constructor
  {
    math::Boxd box(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(1.0, 2.0, 5.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(1.0, 2.0, 5.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }

  // Vector Dimension and mat constructor
  {
    math::Boxd box(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(math::Vector3d(2.2, 2.0, 10.0), box.Size());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), box.Material());

    math::Boxd box2(math::Vector3d(2.2, 2.0, 10.0),
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(box, box2);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, Mutators)
{
  math::Boxd box;
  box.SetSize(100.1, 2.3, 5.6);
  box.SetMaterial(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, box.Size().X());
  EXPECT_DOUBLE_EQ(2.3, box.Size().Y());
  EXPECT_DOUBLE_EQ(5.6, box.Size().Z());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), box.Material());

  box.SetSize(math::Vector3d(3.4, 1.2, 0.5));
  EXPECT_DOUBLE_EQ(3.4, box.Size().X());
  EXPECT_DOUBLE_EQ(1.2, box.Size().Y());
  EXPECT_DOUBLE_EQ(0.5, box.Size().Z());
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Boxd box(1.0, 0.1, 10.4);
  double expectedVolume = 1.0 * 0.1 * 10.4;
  EXPECT_DOUBLE_EQ(expectedVolume, box.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, box.DensityFromMass(mass));

  // Bad density
  math::Boxd box2;
  EXPECT_GT(0.0, box2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(BoxTest, Intersections)
{
  // No intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_EQ(box.Intersections(plane).size(), 0UL);
  }

  // Plane crosses 4 edges
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(4UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, 0.0)), 1UL);
  }

  // Plane coincides with box's face
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 1.0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(4UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, 1.0)), 1UL);
  }

  // 3 intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 1.0);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(3UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, -1.0)), 1UL);
  }

  // 6 intersections
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 0.5);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(6UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.5)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 0.5, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.5)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(0.5, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 0.5, -1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(0.5, 1.0, -1.0)), 1UL);
  }

  // 5 intersections
  // This is the plane above tilted further up
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 2.0), 0.5);

    auto intersections = box.Intersections(plane);
    ASSERT_EQ(5UL, intersections.size());
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, 1.0, 0.25)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-1.0, -0.5, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, -1.0, 0.25)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(-0.5, -1.0, 1.0)), 1UL);
    EXPECT_EQ(intersections.count(math::Vector3d(1.0, 1.0, -0.75)), 1UL);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeBelow)
{
  // Fully above
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_DOUBLE_EQ(0.0, box.VolumeBelow(plane));
  }
  // Fully below
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 20.0);
    EXPECT_DOUBLE_EQ(box.Volume(), box.VolumeBelow(plane));
  }
  // Fully below (because plane is rotated down)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 20.0);
    EXPECT_DOUBLE_EQ(box.Volume(), box.VolumeBelow(plane));
  }
  // Cut in half
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 1, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(-1, 0, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(-1, -1, 0), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 1, 1), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1, 1, 1), 0);

    EXPECT_DOUBLE_EQ(box.Volume()/2, box.VolumeBelow(plane));
  }
  // Cut in 3/4
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0.5);

    EXPECT_DOUBLE_EQ(3*box.Volume()/4, box.VolumeBelow(plane));
  }
  // Opposites add to the total volume
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(0, 0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(0, 0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(0, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(0, 1.0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(-1, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(-1, 1.0, 1.0), -0.5);

    EXPECT_DOUBLE_EQ(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB));
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, CenterOfVolumeBelow)
{
  // Fully above
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_FALSE(box.CenterOfVolumeBelow(plane).has_value());
  }

  // Fully below
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 5.0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane), math::Vector3d(0, 0, 0));
  }

  // Cut in half
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane).value(),
      math::Vector3d(0, 0, -0.5));
  }

  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 0);
    EXPECT_TRUE(box.CenterOfVolumeBelow(plane).has_value());
    EXPECT_EQ(box.CenterOfVolumeBelow(plane).value(),
      math::Vector3d(0, 0, 0.5));
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, VerticesBelow)
{
  math::Boxd box(2.0, 2.0, 2.0);
  auto size = box.Size();

  math::Vector3d pXpYpZ{ size.X()/2,  size.Y()/2,  size.Z()/2};
  math::Vector3d nXpYpZ{-size.X()/2,  size.Y()/2,  size.Z()/2};
  math::Vector3d pXnYpZ{ size.X()/2, -size.Y()/2,  size.Z()/2};
  math::Vector3d nXnYpZ{-size.X()/2, -size.Y()/2,  size.Z()/2};
  math::Vector3d pXpYnZ{ size.X()/2,  size.Y()/2, -size.Z()/2};
  math::Vector3d nXpYnZ{-size.X()/2,  size.Y()/2, -size.Z()/2};
  math::Vector3d pXnYnZ{ size.X()/2, -size.Y()/2, -size.Z()/2};
  math::Vector3d nXnYnZ{-size.X()/2, -size.Y()/2, -size.Z()/2};

  // Fully above
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), -5.0);
    EXPECT_TRUE(box.VerticesBelow(plane).empty());
  }
  // Fully below
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 20.0);
    EXPECT_EQ(8u, box.VerticesBelow(plane).size());
  }
  // Fully below (because plane is rotated down)
  {
    math::Planed plane(math::Vector3d(0.0, 0.0, -1.0), 20.0);
    EXPECT_EQ(8u, box.VerticesBelow(plane).size());
  }
  // 4 vertices
  {
    math::Planed plane(math::Vector3d(0, 0, 1.0), 0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(0, 1, 0), 0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(-1, 0, 0), -0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(1, 1, 1), 0.0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(4u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
  }
  // 6 vertices
  {
    math::Planed plane(math::Vector3d(-1, -1, 0), 0.3);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(6u, vertices.size());

    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  {
    math::Planed plane(math::Vector3d(0, 1, 1), 0.9);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(6u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  // 2 vertices
  {
    math::Planed plane(math::Vector3d(-1, -1, 0), -0.5);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(2u, vertices.size());

    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYpZ), 1UL);
  }
  // 7 vertices
  {
    math::Planed plane(math::Vector3d(1, 1, 1), 1.0);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(7u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYpZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYnZ), 1UL);
    EXPECT_EQ(vertices.count(nXpYpZ), 1UL);
    EXPECT_EQ(vertices.count(pXnYnZ), 1UL);
    EXPECT_EQ(vertices.count(pXpYnZ), 1UL);
  }
  // 1 vertex
  {
    math::Planed plane(math::Vector3d(1, 1, 1), -1.2);

    auto vertices = box.VerticesBelow(plane);
    ASSERT_EQ(1u, vertices.size());

    EXPECT_EQ(vertices.count(nXnYnZ), 1UL);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, VolumeBelowDiagonalPlanes)
{
  // Tetrahedron corner cut: Box 2x2x2, plane n=(1,1,1), d=-1.5
  // alpha = -1.5 + 1 + 1 + 1 = 1.5 < min(M_i)=2, so pure tetrahedron
  // V = totalVol * alpha^3 / (6*M1*M2*M3) = 8 * 1.5^3 / (6*2*2*2)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), -1.5);
    double alpha = 1.5;
    double expected = 8.0 * alpha * alpha * alpha / (6.0 * 2.0 * 2.0 * 2.0);
    EXPECT_NEAR(expected, box.VolumeBelow(plane), 1e-10);
  }

  // Non-unit normal: plane n=(2,1,3), d=1.0, box 4x2x6
  // m1=2, m2=1, m3=3, h1=2, h2=1, h3=3
  // M1=2*4=8, M2=1*2=2, M3=3*6=18
  // alpha = 1 + 2*2 + 1*1 + 3*3 = 15
  // IE3(15) = 15^3 - max(0,15-8)^3 - max(0,15-2)^3 - max(0,15-18)^3
  //         + max(0,15-8-2)^3 + max(0,15-8-18)^3 + max(0,15-2-18)^3
  //         - max(0,15-8-2-18)^3
  //         = 3375 - 343 - 2197 - 0 + 125 + 0 + 0 - 0 = 960
  // V = 48 * 960 / (6*8*2*18)
  {
    math::Boxd box(4.0, 2.0, 6.0);
    math::Planed plane(math::Vector3d(2.0, 1.0, 3.0), 1.0);
    double expected = 48.0 * 960.0 / (6.0 * 2.0 * 8.0 * 18.0);
    EXPECT_NEAR(expected, box.VolumeBelow(plane), 1e-10);
  }

  // Asymmetric box 1x2x3, plane n=(1,1,1), d=0
  // m=1,1,1. h=0.5,1,1.5. M=1,2,3.
  // alpha = 0 + 0.5 + 1 + 1.5 = 3 = M_sum/2 -> half volume
  {
    math::Boxd box(1.0, 2.0, 3.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 0.0);
    EXPECT_NEAR(box.Volume() / 2.0, box.VolumeBelow(plane), 1e-10);
  }

  // Plane tangent to edge: alpha = M1
  // Box 2x2x2, n=(1,0,0), d=-1. m1=1,m2=0,m3=0. alpha=d+m1*h1= -1+1=0
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 0.0, 0.0), -1.0);
    EXPECT_NEAR(0.0, box.VolumeBelow(plane), 1e-10);
  }

  // Very small box: 0.002x0.002x0.002, plane through center
  // This exposes the WellOrderedVectors 1e-3 tolerance bug in old code
  {
    math::Boxd box(0.002, 0.002, 0.002);
    math::Planed plane(math::Vector3d(0.0, 0.0, 1.0), 0.0);
    EXPECT_NEAR(box.Volume() / 2.0, box.VolumeBelow(plane), 1e-15);
  }

  // Complementary planes: V(n,d) + V(-n,-d) == totalVolume for diagonal
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(1.0, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(-1.0, -1.0, -1.0), -0.5);
    EXPECT_NEAR(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB), 1e-10);
  }

  // Non-symmetric complementary
  {
    math::Boxd box(1.0, 2.0, 3.0);
    math::Planed planeA(math::Vector3d(2.0, -1.0, 1.0), 0.3);
    math::Planed planeB(math::Vector3d(-2.0, 1.0, -1.0), -0.3);
    EXPECT_NEAR(box.Volume(),
        box.VolumeBelow(planeA) + box.VolumeBelow(planeB), 1e-10);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, CenterOfVolumeBelowDiagonal)
{
  // Centroid direction: for plane through origin with diagonal normal,
  // n . centroid < 0 (centroid is on the "below" side)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 0.0);
    auto cov = box.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    double dot = math::Vector3d(1, 1, 1).Dot(cov.value());
    EXPECT_LT(dot, 0.0);
  }

  // Tetrahedron corner: Box 2x2x2, plane n=(1,1,1), d=-1.5
  // alpha = 1.5 < min(M_i) = 2, so the clipped region is a tetrahedron.
  // Vertices: (-1,-1,-1), (0.5,-1,-1), (-1,0.5,-1), (-1,-1,0.5)
  // Tet centroid = average of 4 vertices = (-0.625, -0.625, -0.625)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), -1.5);
    auto cov = box.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(-0.625, cov.value().X(), 1e-10);
    EXPECT_NEAR(-0.625, cov.value().Y(), 1e-10);
    EXPECT_NEAR(-0.625, cov.value().Z(), 1e-10);
  }

  // Diagonal half-cut: Box 2x2x2, plane x+y+z=0
  // By symmetry, centroid = (-c,-c,-c) for some c > 0
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed plane(math::Vector3d(1.0, 1.0, 1.0), 0.0);
    auto cov = box.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    // By symmetry, all coordinates are equal
    EXPECT_NEAR(cov.value().X(), cov.value().Y(), 1e-10);
    EXPECT_NEAR(cov.value().Y(), cov.value().Z(), 1e-10);
    // And negative
    EXPECT_LT(cov.value().X(), 0.0);
  }

  // Weighted sum test: CoV_below * V_below + CoV_above * V_above = (0,0,0)
  // Using complementary planes
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(1.0, 1.0, 1.0), 0.5);
    math::Planed planeB(math::Vector3d(-1.0, -1.0, -1.0), -0.5);

    auto covA = box.CenterOfVolumeBelow(planeA);
    auto covB = box.CenterOfVolumeBelow(planeB);
    ASSERT_TRUE(covA.has_value());
    ASSERT_TRUE(covB.has_value());

    auto volA = box.VolumeBelow(planeA);
    auto volB = box.VolumeBelow(planeB);

    auto weightedSum = covA.value() * volA + covB.value() * volB;
    EXPECT_NEAR(0.0, weightedSum.X(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Y(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Z(), 1e-10);
  }

  // Another weighted sum with asymmetric box
  {
    math::Boxd box(1.0, 2.0, 3.0);
    math::Planed planeA(math::Vector3d(2.0, -1.0, 1.0), 0.3);
    math::Planed planeB(math::Vector3d(-2.0, 1.0, -1.0), -0.3);

    auto covA = box.CenterOfVolumeBelow(planeA);
    auto covB = box.CenterOfVolumeBelow(planeB);
    ASSERT_TRUE(covA.has_value());
    ASSERT_TRUE(covB.has_value());

    auto volA = box.VolumeBelow(planeA);
    auto volB = box.VolumeBelow(planeB);

    auto weightedSum = covA.value() * volA + covB.value() * volB;
    EXPECT_NEAR(0.0, weightedSum.X(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Y(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Z(), 1e-10);
  }

  // 2D normal (nonzero == 2): exercises the k=2 centroid path
  // n=(1,1,0), d=0.5 on box 2x2x2
  // z-centroid should be 0 (plane is parallel to z-axis)
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(1.0, 1.0, 0.0), 0.5);
    math::Planed planeB(math::Vector3d(-1.0, -1.0, 0.0), -0.5);

    auto covA = box.CenterOfVolumeBelow(planeA);
    auto covB = box.CenterOfVolumeBelow(planeB);
    ASSERT_TRUE(covA.has_value());
    ASSERT_TRUE(covB.has_value());

    // z-centroid must be 0 for both (plane has no z-component)
    EXPECT_NEAR(0.0, covA.value().Z(), 1e-10);
    EXPECT_NEAR(0.0, covB.value().Z(), 1e-10);

    // Weighted sum conservation
    auto volA = box.VolumeBelow(planeA);
    auto volB = box.VolumeBelow(planeB);
    auto ws = covA.value() * volA + covB.value() * volB;
    EXPECT_NEAR(0.0, ws.X(), 1e-10);
    EXPECT_NEAR(0.0, ws.Y(), 1e-10);
    EXPECT_NEAR(0.0, ws.Z(), 1e-10);
  }

  // Flipped 3D normal: n=(-1,1,-1), exercises sign transform for centroid
  {
    math::Boxd box(2.0, 2.0, 2.0);
    math::Planed planeA(math::Vector3d(-1.0, 1.0, -1.0), 0.3);
    math::Planed planeB(math::Vector3d(1.0, -1.0, 1.0), -0.3);

    auto covA = box.CenterOfVolumeBelow(planeA);
    auto covB = box.CenterOfVolumeBelow(planeB);
    ASSERT_TRUE(covA.has_value());
    ASSERT_TRUE(covB.has_value());

    // centroid should be on the "below" side: n . centroid < 0
    EXPECT_LT(math::Vector3d(-1, 1, -1).Dot(covA.value()), 0.0);

    // Weighted sum conservation
    auto volA = box.VolumeBelow(planeA);
    auto volB = box.VolumeBelow(planeB);
    auto ws = covA.value() * volA + covB.value() * volB;
    EXPECT_NEAR(0.0, ws.X(), 1e-10);
    EXPECT_NEAR(0.0, ws.Y(), 1e-10);
    EXPECT_NEAR(0.0, ws.Z(), 1e-10);
  }
}

//////////////////////////////////////////////////
TEST(BoxTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double w = 0.1;
  double h = 34.12;
  math::Boxd box(l, w, h);
  box.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixx = (1.0/12.0) * mass * (w*w + h*h);
  double iyy = (1.0/12.0) * mass * (l*l + h*h);
  double izz = (1.0/12.0) * mass * (l*l + w*w);

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixx, iyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  box.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());
}
