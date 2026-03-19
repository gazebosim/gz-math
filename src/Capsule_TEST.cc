/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "gz/math/Capsule.hh"
#include "gz/math/Helpers.hh"
#include "gz/math/Plane.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(CapsuleTest, Constructor)
{
  // Default constructor
  {
    math::Capsuled capsule;
    EXPECT_DOUBLE_EQ(0.0, capsule.Length());
    EXPECT_DOUBLE_EQ(0.0, capsule.Radius());
    EXPECT_EQ(math::Material(), capsule.Mat());

    math::Capsuled capsule2;
    EXPECT_EQ(capsule, capsule2);
  }

  // Length and radius constructor
  {
    math::Capsuled capsule(1.0, 2.0);
    EXPECT_DOUBLE_EQ(1.0, capsule.Length());
    EXPECT_DOUBLE_EQ(2.0, capsule.Radius());
    EXPECT_EQ(math::Material(), capsule.Mat());

    math::Capsuled capsule2(1.0, 2.0);
    EXPECT_EQ(capsule, capsule2);
  }

  // Length, radius, mat
  {
    math::Capsuled capsule(1.0, 2.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_DOUBLE_EQ(1.0, capsule.Length());
    EXPECT_DOUBLE_EQ(2.0, capsule.Radius());
    EXPECT_EQ(math::Material(math::MaterialType::WOOD), capsule.Mat());

    math::Capsuled capsule2(1.0, 2.0,
        math::Material(math::MaterialType::WOOD));
    EXPECT_EQ(capsule, capsule2);
  }
}

//////////////////////////////////////////////////
TEST(CapsuleTest, Mutators)
{
  math::Capsuled capsule;
  EXPECT_DOUBLE_EQ(0.0, capsule.Length());
  EXPECT_DOUBLE_EQ(0.0, capsule.Radius());
  EXPECT_EQ(math::Material(), capsule.Mat());

  capsule.SetLength(100.1);
  capsule.SetRadius(.123);
  capsule.SetMat(math::Material(math::MaterialType::PINE));

  EXPECT_DOUBLE_EQ(100.1, capsule.Length());
  EXPECT_DOUBLE_EQ(.123, capsule.Radius());
  EXPECT_EQ(math::Material(math::MaterialType::PINE), capsule.Mat());
}

//////////////////////////////////////////////////
TEST(CapsuleTest, VolumeAndDensity)
{
  double mass = 1.0;
  math::Capsuled capsule(1.0, 0.001);
  double expectedVolume = (GZ_PI * std::pow(0.001, 2) * (1.0 + 4./3. * 0.001));
  EXPECT_DOUBLE_EQ(expectedVolume, capsule.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, capsule.DensityFromMass(mass));

  // Bad density
  math::Capsuled capsule2;
  EXPECT_TRUE(math::isnan(capsule2.DensityFromMass(mass)));
}

//////////////////////////////////////////////////
TEST(CapsuleTest, Mass)
{
  double mass = 2.0;
  double l = 2.0;
  double r = 0.1;
  math::Capsuled capsule(l, r);
  capsule.SetDensityFromMass(mass);

  const double cylinderVolume = GZ_PI * r*r * l;
  const double sphereVolume = GZ_PI * 4. / 3. * r*r*r;
  const double volume = cylinderVolume + sphereVolume;
  const double cylinderMass = mass * cylinderVolume / volume;
  const double sphereMass = mass * sphereVolume / volume;

  // expected values based on formula used in Open Dynamics Engine
  // https://bitbucket.org/odedevs/ode/src/0.16.2/ode/src/mass.cpp#lines-148:153
  // and the following article:
  // https://www.gamedev.net/tutorials/_/technical/math-and-physics/capsule-inertia-tensor-r3856/
  double ixxIyy = (1/12.0) * cylinderMass * (3*r*r + l*l)
    + sphereMass * (0.4*r*r + 0.375*r*l + 0.25*l*l);
  double izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass);

  math::MassMatrix3d expectedMassMat;
  expectedMassMat.SetInertiaMatrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
  expectedMassMat.SetMass(mass);

  auto massMat = capsule.MassMatrix();
  ASSERT_NE(std::nullopt, massMat);
  EXPECT_EQ(expectedMassMat, *massMat);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMat->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat->Mass());
}

//////////////////////////////////////////////////
TEST(CapsuleTest, VolumeBelow)
{
  // Capsule: length=4, radius=1, centered at origin, Z-aligned
  // Total height = 4 + 2*1 = 6 (from z=-3 to z=3)
  {
    math::Capsuled cap(4.0, 1.0);

    // Fully above
    math::Planed planeAbove(math::Vector3d(0, 0, 1), -4.0);
    EXPECT_NEAR(0.0, cap.VolumeBelow(planeAbove), 1e-10);

    // Fully below
    math::Planed planeBelow(math::Vector3d(0, 0, 1), 4.0);
    EXPECT_NEAR(cap.Volume(), cap.VolumeBelow(planeBelow), 1e-10);

    // Horizontal plane through center: half volume
    math::Planed planeCenter(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_NEAR(cap.Volume() / 2, cap.VolumeBelow(planeCenter), 1e-10);
  }

  // Plane cutting only through hemisphere cap
  {
    math::Capsuled cap(4.0, 1.0);
    // Plane at z = -2.5 cuts through bottom hemisphere only
    // (hemisphere center at z=-2, hemisphere goes from z=-3 to z=-2)
    // In hemisphere local frame: plane at z = -0.5
    math::Planed plane(math::Vector3d(0, 0, 1), -2.5);
    auto vol = cap.VolumeBelow(plane);
    EXPECT_GT(vol, 0);
    // Should be a sphere cap with h = 1 - 0.5 = 0.5
    double h = 0.5;
    double expectedCapVol = GZ_PI * h * h * (3.0 - h) / 3.0;
    EXPECT_NEAR(expectedCapVol, vol, 1e-10);
  }

  // Complementary planes
  {
    math::Capsuled cap(4.0, 1.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed p1(normal, offset);
    math::Planed p2(-normal, -offset);

    EXPECT_NEAR(cap.Volume(),
                cap.VolumeBelow(p1) + cap.VolumeBelow(p2), 1e-10);
  }

  // Diagonal plane through center: half volume
  {
    math::Capsuled cap(4.0, 1.0);
    math::Vector3d normal(1, 0, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);
    EXPECT_NEAR(cap.Volume() / 2, cap.VolumeBelow(plane), 1e-10);
  }

  // Invalid capsule
  {
    math::Capsuled cap(0.0, 0.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_DOUBLE_EQ(0.0, cap.VolumeBelow(plane));
  }
}

//////////////////////////////////////////////////
TEST(CapsuleTest, CenterOfVolumeBelow)
{
  // Fully below: centroid at origin
  {
    math::Capsuled cap(4.0, 1.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 4.0);
    auto cov = cap.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    EXPECT_NEAR(0.0, cov->Z(), 1e-10);
  }

  // Fully above: nullopt
  {
    math::Capsuled cap(4.0, 1.0);
    math::Planed plane(math::Vector3d(0, 0, 1), -4.0);
    EXPECT_FALSE(cap.CenterOfVolumeBelow(plane).has_value());
  }

  // Centroid direction: for plane through origin, n . centroid < 0
  {
    math::Capsuled cap(4.0, 1.0);
    math::Vector3d normal(1, 1, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);
    auto cov = cap.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_LT(normal.Dot(*cov), 0);
  }

  // Weighted sum: CoV_below * V_below + CoV_above * V_above == (0,0,0)
  {
    math::Capsuled cap(4.0, 1.0);
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed plane(normal, offset);
    math::Planed flipped(-normal, -offset);

    auto vBelow = cap.VolumeBelow(plane);
    auto vAbove = cap.VolumeBelow(flipped);
    auto covBelow = cap.CenterOfVolumeBelow(plane);
    auto covAbove = cap.CenterOfVolumeBelow(flipped);

    ASSERT_TRUE(covBelow.has_value());
    ASSERT_TRUE(covAbove.has_value());

    auto ws = (*covBelow) * vBelow + (*covAbove) * vAbove;
    EXPECT_NEAR(0.0, ws.X(), 1e-6);
    EXPECT_NEAR(0.0, ws.Y(), 1e-6);
    EXPECT_NEAR(0.0, ws.Z(), 1e-6);
  }

  // Invalid capsule
  {
    math::Capsuled cap(0.0, 0.0);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_FALSE(cap.CenterOfVolumeBelow(plane).has_value());
  }
}

//////////////////////////////////////////////////
TEST(CapsuleTest, VolumeBelowFloat)
{
  float length = 2.0f;
  float r = 1.0f;
  math::Capsule<float> cap(length, r);

  // Horizontal plane at z=0: half volume by symmetry
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 0.0f);
    EXPECT_NEAR(cap.Volume() / 2.0f, cap.VolumeBelow(plane), 1e-3f);
  }

  // Fully below
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 10.0f);
    EXPECT_NEAR(cap.Volume(), cap.VolumeBelow(plane), 1e-3f);
  }

  // Fully above
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, -10.0f);
    EXPECT_NEAR(0.0f, cap.VolumeBelow(plane), 1e-3f);
  }

  // CenterOfVolumeBelow with float
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 0.0f);
    auto cov = cap.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0f, cov.value().X(), 1e-3f);
    EXPECT_NEAR(0.0f, cov.value().Y(), 1e-3f);
    // Bottom half centroid is below z=0
    EXPECT_TRUE(cov.value().Z() < 0.0f);
  }
}
