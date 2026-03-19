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

#include "gz/math/Cylinder.hh"
#include "gz/math/Plane.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(CylinderTest, Constructor)
{
  // Default constructor
  {
    math::Cylinderd cylinder;
    EXPECT_DOUBLE_EQ(0.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(0.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2;
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length and radius constructor
  {
    math::Cylinderd cylinder(1.0, 2.0);
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0);
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length, radius, and rot constructor
  {
    math::Cylinderd cylinder(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cylinder, cylinder2);
  }

  // Length, radius, mat and rot constructor
  {
    math::Cylinderd cylinder(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cylinder.Length());
    EXPECT_DOUBLE_EQ(2.0, cylinder.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cylinder.RotationalOffset());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), cylinder.Mat());

    math::Cylinderd cylinder2(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cylinder, cylinder2);
  }
}

//////////////////////////////////////////////////
TEST(CylinderTest, Mutators)
{
  math::Cylinderd cylinder;
  EXPECT_DOUBLE_EQ(0.0, cylinder.Length());
  EXPECT_DOUBLE_EQ(0.0, cylinder.Radius());
  EXPECT_EQ(math::Quaterniond::Identity, cylinder.RotationalOffset());
  EXPECT_EQ(math::Material(), cylinder.Mat());

  cylinder.SetLength(100.1);
  cylinder.SetRadius(.123);
  cylinder.SetRotationalOffset(math::Quaterniond(1.2, 2.3, 3.4));
  cylinder.SetMat(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, cylinder.Length());
  EXPECT_DOUBLE_EQ(.123, cylinder.Radius());
  EXPECT_EQ(math::Quaterniond(1.2, 2.3, 3.4),
    cylinder.RotationalOffset());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), cylinder.Mat());
}

//////////////////////////////////////////////////
TEST(CylinderTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Cylinderd cylinder(1.0, 0.001);
  double expectedVolume = (GZ_PI * std::pow(0.001, 2) * 1.0);
  EXPECT_DOUBLE_EQ(expectedVolume, cylinder.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, cylinder.DensityFromMass(mass));

  // Bad density
  math::Cylinderd cylinder2;
  EXPECT_GT(0.0, cylinder2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(CylinderTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double r = 0.1;
  math::Cylinderd cylinder(l, r);
  cylinder.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixxIyy = (1/12.0) * mass * (3*r*r + l*l);
  double izz = 0.5 * mass * r * r;

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  cylinder.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());

  auto massMatOpt = cylinder.MassMatrix();
  ASSERT_NE(std::nullopt, massMatOpt);
  EXPECT_EQ(expectedMassMat, *massMatOpt);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMatOpt->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMatOpt->Mass());
}

//////////////////////////////////////////////////
TEST(CylinderTest, VolumeBelow)
{
  // Cylinder: length=4, radius=2, centered at origin, Z-aligned
  {
    math::Cylinderd cyl(4.0, 2.0);

    // Horizontal plane: fully above
    math::Planed planeAbove(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_NEAR(0.0, cyl.VolumeBelow(planeAbove), 1e-10);

    // Horizontal plane: fully below
    math::Planed planeBelow(math::Vector3d(0, 0, 1), 3.0);
    EXPECT_NEAR(cyl.Volume(), cyl.VolumeBelow(planeBelow), 1e-10);

    // Horizontal plane through center: half volume
    math::Planed planeCenter(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_NEAR(cyl.Volume() / 2, cyl.VolumeBelow(planeCenter), 1e-10);

    // Horizontal plane at z = -1 (bottom quarter)
    math::Planed planeQuarter(math::Vector3d(0, 0, 1), -1.0);
    EXPECT_NEAR(cyl.Volume() / 4, cyl.VolumeBelow(planeQuarter), 1e-10);
  }

  // Vertical plane through axis: half volume
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Planed planeV(math::Vector3d(1, 0, 0), 0.0);
    EXPECT_NEAR(cyl.Volume() / 2, cyl.VolumeBelow(planeV), 1e-10);

    // Vertical plane tangent to side: full volume
    math::Planed planeVTangent(math::Vector3d(1, 0, 0), 2.0);
    EXPECT_NEAR(cyl.Volume(), cyl.VolumeBelow(planeVTangent), 1e-10);

    // Vertical plane beyond cylinder: zero
    math::Planed planeVOut(math::Vector3d(1, 0, 0), -3.0);
    EXPECT_NEAR(0.0, cyl.VolumeBelow(planeVOut), 1e-10);
  }

  // Diagonal plane through center: half volume
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Vector3d normal(1, 0, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);
    EXPECT_NEAR(cyl.Volume() / 2, cyl.VolumeBelow(plane), 1e-10);
  }

  // Complementary planes: V(n,d) + V(-n,-d) == totalVolume
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed p1(normal, offset);
    math::Planed p2(-normal, -offset);

    EXPECT_NEAR(cyl.Volume(),
                cyl.VolumeBelow(p1) + cyl.VolumeBelow(p2), 1e-10);
  }

  // Non-unit normal
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 5), 0.0);
    EXPECT_NEAR(cyl.Volume() / 2, cyl.VolumeBelow(plane), 1e-10);
  }

  // Invalid cylinder
  {
    math::Cylinderd cyl(0.0, 0.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_DOUBLE_EQ(0.0, cyl.VolumeBelow(plane));
  }
}

//////////////////////////////////////////////////
TEST(CylinderTest, CenterOfVolumeBelow)
{
  // Fully below: centroid at origin
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 3.0);
    auto cov = cyl.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    EXPECT_NEAR(0.0, cov->Z(), 1e-10);
  }

  // Fully above: nullopt
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_FALSE(cyl.CenterOfVolumeBelow(plane).has_value());
  }

  // Horizontal plane through center: centroid at (0,0,-1)
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    auto cov = cyl.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    EXPECT_NEAR(-1.0, cov->Z(), 1e-10);
  }

  // Vertical plane through axis: centroid at (-4r/(3pi), 0, 0)
  {
    double r = 2.0;
    math::Cylinderd cyl(4.0, r);
    math::Planed plane(math::Vector3d(1, 0, 0), 0.0);
    auto cov = cyl.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    double expectedCx = -4.0 * r / (3.0 * GZ_PI);
    EXPECT_NEAR(expectedCx, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    EXPECT_NEAR(0.0, cov->Z(), 1e-10);
  }

  // Centroid direction: for plane through origin, n . centroid < 0
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Vector3d normal(1, 1, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);
    auto cov = cyl.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_LT(normal.Dot(*cov), 0);
  }

  // Weighted sum: CoV_below * V_below + CoV_above * V_above == (0,0,0)
  {
    math::Cylinderd cyl(4.0, 2.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed plane(normal, offset);
    math::Planed flipped(-normal, -offset);

    auto vBelow = cyl.VolumeBelow(plane);
    auto vAbove = cyl.VolumeBelow(flipped);
    auto covBelow = cyl.CenterOfVolumeBelow(plane);
    auto covAbove = cyl.CenterOfVolumeBelow(flipped);

    ASSERT_TRUE(covBelow.has_value());
    ASSERT_TRUE(covAbove.has_value());

    auto ws = (*covBelow) * vBelow + (*covAbove) * vAbove;
    EXPECT_NEAR(0.0, ws.X(), 1e-6);
    EXPECT_NEAR(0.0, ws.Y(), 1e-6);
    EXPECT_NEAR(0.0, ws.Z(), 1e-6);
  }

  // With rotational offset
  {
    // Rotate 90 deg around Y: cylinder axis becomes X
    math::Quaterniond rot(math::Vector3d(0, 1, 0), GZ_PI / 2);
    math::Cylinderd cyl(4.0, 2.0, rot);

    // Horizontal plane through center: should now split along X axis
    math::Planed plane(math::Vector3d(1, 0, 0), 0.0);
    EXPECT_NEAR(cyl.Volume() / 2, cyl.VolumeBelow(plane), 1e-10);
  }
}

//////////////////////////////////////////////////
TEST(CylinderTest, VolumeBelowFloat)
{
  float length = 4.0f;
  float r = 2.0f;
  math::Cylinder<float> cyl(length, r);

  // Horizontal plane at z=0: half volume
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 0.0f);
    EXPECT_NEAR(cyl.Volume() / 2.0f, cyl.VolumeBelow(plane), 1e-3f);
  }

  // Fully below
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 10.0f);
    EXPECT_NEAR(cyl.Volume(), cyl.VolumeBelow(plane), 1e-3f);
  }

  // Fully above
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, -10.0f);
    EXPECT_NEAR(0.0f, cyl.VolumeBelow(plane), 1e-3f);
  }

  // CenterOfVolumeBelow with float
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 0.0f);
    auto cov = cyl.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0f, cov.value().X(), 1e-3f);
    EXPECT_NEAR(0.0f, cov.value().Y(), 1e-3f);
    EXPECT_NEAR(-1.0f, cov.value().Z(), 1e-3f);
  }
}
