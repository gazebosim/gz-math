/*
 * Copyright 2024 CogniPilot Foundation
 * Copyright 2024 Open Source Robotics Foundation
 * Copyright 2024 Rudis Laboratories
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

#include "gz/math/Cone.hh"
#include "gz/math/Plane.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(ConeTest, Constructor)
{
  // Default constructor
  {
    math::Coned cone;
    EXPECT_DOUBLE_EQ(0.0, cone.Length());
    EXPECT_DOUBLE_EQ(0.0, cone.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cone.RotationalOffset());
    EXPECT_EQ(math::Material(), cone.Mat());

    math::Coned cone2;
    EXPECT_EQ(cone, cone2);
  }

  // Length and radius constructor
  {
    math::Coned cone(1.0, 2.0);
    EXPECT_DOUBLE_EQ(1.0, cone.Length());
    EXPECT_DOUBLE_EQ(2.0, cone.Radius());
    EXPECT_EQ(math::Quaterniond::Identity, cone.RotationalOffset());
    EXPECT_EQ(math::Material(), cone.Mat());

    math::Coned cone2(1.0, 2.0);
    EXPECT_EQ(cone, cone2);
  }

  // Length, radius, and rot constructor
  {
    math::Coned cone(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cone.Length());
    EXPECT_DOUBLE_EQ(2.0, cone.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cone.RotationalOffset());
    EXPECT_EQ(math::Material(), cone.Mat());

    math::Coned cone2(1.0, 2.0, math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cone, cone2);
  }

  // Length, radius, mat and rot constructor
  {
    math::Coned cone(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_DOUBLE_EQ(1.0, cone.Length());
    EXPECT_DOUBLE_EQ(2.0, cone.Radius());
    EXPECT_EQ(math::Quaterniond(0.1, 0.2, 0.3),
        cone.RotationalOffset());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), cone.Mat());

    math::Coned cone2(1.0, 2.0,
        math::Material(math::MaterialType::WOOD),
        math::Quaterniond(0.1, 0.2, 0.3));
    EXPECT_EQ(cone, cone2);
  }
}

//////////////////////////////////////////////////
TEST(ConeTest, Mutators)
{
  math::Coned cone;
  EXPECT_DOUBLE_EQ(0.0, cone.Length());
  EXPECT_DOUBLE_EQ(0.0, cone.Radius());
  EXPECT_EQ(math::Quaterniond::Identity, cone.RotationalOffset());
  EXPECT_EQ(math::Material(), cone.Mat());

  cone.SetLength(100.1);
  cone.SetRadius(.123);
  cone.SetRotationalOffset(math::Quaterniond(1.2, 2.3, 3.4));
  cone.SetMat(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, cone.Length());
  EXPECT_DOUBLE_EQ(.123, cone.Radius());
  EXPECT_EQ(math::Quaterniond(1.2, 2.3, 3.4),
    cone.RotationalOffset());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), cone.Mat());
}

//////////////////////////////////////////////////
TEST(ConeTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Coned cone(1.0, 0.001);
  double expectedVolume = (GZ_PI * std::pow(0.001, 2) * 1.0 / 3.0);
  EXPECT_DOUBLE_EQ(expectedVolume, cone.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, cone.DensityFromMass(mass));

  // Bad density
  math::Coned cone2;
  EXPECT_GT(0.0, cone2.DensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(ConeTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double r = 0.1;
  math::Coned cone(l, r);
  cone.SetDensityFromMass(mass);

  math::MassMatrix3d massMat;
  double ixxIyy = (3.0/80.0) * mass * (4*r*r + l*l);
  double izz = (3.0/10.0) * mass * r * r;

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  cone.MassMatrix(massMat);
  EXPECT_EQ(expectedMassMat, massMat);
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat.Mass());

  auto massMatOpt = cone.MassMatrix();
  ASSERT_NE(std::nullopt, massMatOpt);
  EXPECT_EQ(expectedMassMat, *massMatOpt);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMatOpt->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMatOpt->Mass());
}

//////////////////////////////////////////////////
TEST(ConeTest, VolumeBelow)
{
  // Cone: length=4, radius=2
  // Base at z=-2 (r=2), apex at z=+2 (r=0)
  {
    math::Coned cone(4.0, 2.0);

    // Fully above
    math::Planed planeAbove(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_NEAR(0.0, cone.VolumeBelow(planeAbove), 1e-10);

    // Fully below
    math::Planed planeBelow(math::Vector3d(0, 0, 1), 3.0);
    EXPECT_NEAR(cone.Volume(), cone.VolumeBelow(planeBelow), 1e-10);

    // Horizontal plane through center (z=0):
    // r(0) = 2*(2-0)/4 = 1
    // V = piR^2/(3L^2) * (L^3 - (L/2-0)^3) = pi*4/(3*16) * (64 - 8) = 56pi/48
    double expectedV = GZ_PI * 4.0 / (3.0 * 16.0) * (64.0 - 8.0);
    math::Planed planeMid(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_NEAR(expectedV, cone.VolumeBelow(planeMid), 1e-10);
  }

  // Complementary planes
  {
    math::Coned cone(4.0, 2.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed p1(normal, offset);
    math::Planed p2(-normal, -offset);

    EXPECT_NEAR(cone.Volume(),
                cone.VolumeBelow(p1) + cone.VolumeBelow(p2), 1e-8);
  }

  // Vertical plane through axis: should give half volume (by symmetry)
  {
    math::Coned cone(4.0, 2.0);
    math::Planed plane(math::Vector3d(1, 0, 0), 0.0);
    EXPECT_NEAR(cone.Volume() / 2, cone.VolumeBelow(plane), 1e-10);
  }

  // Diagonal plane through center
  {
    math::Coned cone(4.0, 2.0);
    math::Vector3d normal(1, 0, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);
    // Not exactly half volume (cone is not symmetric about diagonal planes
    // through center), but we can verify complementary property.
    math::Planed flipped(-normal, 0.0);
    EXPECT_NEAR(cone.Volume(),
                cone.VolumeBelow(plane) + cone.VolumeBelow(flipped), 1e-8);
  }

  // Invalid cone
  {
    math::Coned cone(0.0, 0.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_DOUBLE_EQ(0.0, cone.VolumeBelow(plane));
  }
}

//////////////////////////////////////////////////
TEST(ConeTest, CenterOfVolumeBelow)
{
  // Fully below: centroid at origin (geometric center)
  // Wait - the centroid of the full cone is NOT at origin. The centroid of
  // a cone is at h/4 from the base. With base at z=-L/2 and apex at z=+L/2,
  // centroid_z = -L/2 + L/4 = -L/4.
  {
    math::Coned cone(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 3.0);
    auto cov = cone.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    // Centroid of full cone: base at z=-2, apex at z=+2
    // centroid_z = -2 + 4/4 = -1
    EXPECT_NEAR(-1.0, cov->Z(), 1e-8);
  }

  // Fully above: nullopt
  {
    math::Coned cone(4.0, 2.0);
    math::Planed plane(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_FALSE(cone.CenterOfVolumeBelow(plane).has_value());
  }

  // Weighted sum: CoV_below * V_below + CoV_above * V_above ==
  // CoV_total * V_total
  {
    math::Coned cone(4.0, 2.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed plane(normal, offset);
    math::Planed flipped(-normal, -offset);

    auto vBelow = cone.VolumeBelow(plane);
    auto vAbove = cone.VolumeBelow(flipped);
    auto covBelow = cone.CenterOfVolumeBelow(plane);
    auto covAbove = cone.CenterOfVolumeBelow(flipped);

    // Full cone centroid
    math::Planed fullPlane(math::Vector3d(0, 0, 1), 3.0);
    auto covFull = cone.CenterOfVolumeBelow(fullPlane);

    ASSERT_TRUE(covBelow.has_value());
    ASSERT_TRUE(covAbove.has_value());
    ASSERT_TRUE(covFull.has_value());

    auto ws = (*covBelow) * vBelow + (*covAbove) * vAbove;
    auto expected = (*covFull) * cone.Volume();
    EXPECT_NEAR(expected.X(), ws.X(), 1e-6);
    EXPECT_NEAR(expected.Y(), ws.Y(), 1e-6);
    EXPECT_NEAR(expected.Z(), ws.Z(), 1e-6);
  }

  // With rotational offset: vertical plane through axis should give half
  {
    math::Quaterniond rot(math::Vector3d(0, 1, 0), GZ_PI / 2);
    math::Coned cone(4.0, 2.0, rot);
    // After rotation, cone axis is along X. A plane with normal along Y
    // at y=0 passes through the axis and gives half volume by symmetry.
    math::Planed plane(math::Vector3d(0, 1, 0), 0.0);
    EXPECT_NEAR(cone.Volume() / 2, cone.VolumeBelow(plane), 1e-8);
  }
}

//////////////////////////////////////////////////
TEST(ConeTest, VolumeBelowFloat)
{
  float length = 4.0f;
  float r = 2.0f;
  math::Cone<float> cone(length, r);

  // Fully below
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 10.0f);
    EXPECT_NEAR(cone.Volume(), cone.VolumeBelow(plane), 1e-3f);
  }

  // Fully above
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, -10.0f);
    EXPECT_NEAR(0.0f, cone.VolumeBelow(plane), 1e-3f);
  }

  // Vertical plane through axis: half by symmetry
  {
    math::Plane<float> plane(math::Vector3<float>{1, 0, 0}, 0.0f);
    EXPECT_NEAR(cone.Volume() / 2.0f, cone.VolumeBelow(plane), 1e-3f);
  }

  // CenterOfVolumeBelow with float
  {
    math::Plane<float> plane(math::Vector3<float>{1, 0, 0}, 0.0f);
    auto cov = cone.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_TRUE(cov.value().X() < 0.0f);
    EXPECT_NEAR(0.0f, cov.value().Y(), 1e-3f);
  }
}
