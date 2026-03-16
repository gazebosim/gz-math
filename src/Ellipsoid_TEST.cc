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

#include "gz/math/Ellipsoid.hh"
#include "gz/math/Helpers.hh"
#include "gz/math/Plane.hh"
#include "gz/math/Sphere.hh"
#include "gz/math/Vector3.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(EllipsoidTest, Constructor)
{
  // Default constructor
  {
    math::Ellipsoidd ellipsoid;
    EXPECT_EQ(math::Vector3d::Zero, ellipsoid.Radii());
    EXPECT_EQ(math::Material(), ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2;
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }

  // Vector3 of radii constructor
  {
    const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
    math::Ellipsoidd ellipsoid(expectedRadii);
    EXPECT_EQ(expectedRadii, ellipsoid.Radii());
    EXPECT_EQ(math::Material(), ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2(expectedRadii);
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }

  // Vector3 of radii and material
  {
    const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
    const math::Material expectedMaterial(math::MaterialType::WOOD);
    math::Ellipsoidd ellipsoid(expectedRadii, expectedMaterial);
    EXPECT_EQ(expectedRadii, ellipsoid.Radii());
    EXPECT_EQ(expectedMaterial, ellipsoid.Mat());

    math::Ellipsoidd ellipsoid2(expectedRadii, expectedMaterial);
    EXPECT_EQ(ellipsoid, ellipsoid2);
  }
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, Mutators)
{
  math::Ellipsoidd ellipsoid;
  EXPECT_EQ(math::Vector3d::Zero, ellipsoid.Radii());
  EXPECT_EQ(math::Material(), ellipsoid.Mat());

  const math::Vector3d expectedRadii(1.0, 2.0, 3.0);
  ellipsoid.SetRadii(expectedRadii);

  const math::Material expectedMaterial(math::MaterialType::PINE);
  ellipsoid.SetMat(expectedMaterial);

  EXPECT_EQ(expectedRadii, ellipsoid.Radii());
  EXPECT_EQ(expectedMaterial, ellipsoid.Mat());
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, VolumeAndDensity)
{
  double mass = 1.0;
  // Basic sphere
  math::Ellipsoidd ellipsoid(2. * math::Vector3d::One);

  double expectedVolume = (4. / 3.) * GZ_PI * std::pow(2.0, 3);
  EXPECT_DOUBLE_EQ(expectedVolume, ellipsoid.Volume());

  double expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, ellipsoid.DensityFromMass(mass));

  math::Ellipsoidd ellipsoid2(math::Vector3d(1, 10, 100));
  expectedVolume = (4. / 3.) * GZ_PI * 1. * 10. * 100.;
  EXPECT_DOUBLE_EQ(expectedVolume, ellipsoid2.Volume());

  expectedDensity = mass / expectedVolume;
  EXPECT_DOUBLE_EQ(expectedDensity, ellipsoid2.DensityFromMass(mass));

  // Check bad cases
  math::Ellipsoidd ellipsoid3(math::Vector3d::Zero);
  EXPECT_FALSE(ellipsoid3.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid4(-math::Vector3d::One);
  EXPECT_FALSE(ellipsoid4.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid5(math::Vector3d(-1, 1, 1));
  EXPECT_FALSE(ellipsoid5.SetDensityFromMass(mass));

  math::Ellipsoidd ellipsoid6(math::Vector3d(-1, -1, 1));
  EXPECT_FALSE(ellipsoid6.SetDensityFromMass(mass));
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, Mass)
{
  const double mass = 2.0;
  math::Ellipsoidd ellipsoid(math::Vector3d(1, 10, 100));
  ellipsoid.SetDensityFromMass(mass);

  const double ixx = (mass / 5.0) * (10. * 10. + 100. * 100.);
  const double iyy = (mass / 5.0) * (1. * 1. + 100. * 100.);
  const double izz = (mass / 5.0) * (1. * 1. + 10. * 10.);
  math::MassMatrix3d expectedMassMat(
    mass, math::Vector3d(ixx, iyy, izz), math::Vector3d::Zero);

  const auto massMat = ellipsoid.MassMatrix();
  ASSERT_NE(std::nullopt, massMat);
  EXPECT_EQ(expectedMassMat, *massMat);
  EXPECT_EQ(expectedMassMat.DiagonalMoments(), massMat->DiagonalMoments());
  EXPECT_DOUBLE_EQ(expectedMassMat.Mass(), massMat->Mass());

  // Zero case
  const math::Ellipsoidd ellipsoid2;
  EXPECT_EQ(std::nullopt, ellipsoid2.MassMatrix());

  // Check bad cases
  const math::Ellipsoidd ellipsoid3(-math::Vector3d::One);
  EXPECT_EQ(std::nullopt, ellipsoid3.MassMatrix());

  const math::Ellipsoidd ellipsoid4(math::Vector3d(-1, 1, 1));
  EXPECT_EQ(std::nullopt, ellipsoid4.MassMatrix());

  const math::Ellipsoidd ellipsoid5(math::Vector3d(-1, -1, 1));
  EXPECT_EQ(std::nullopt, ellipsoid5.MassMatrix());
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, VolumeBelow)
{
  // Sphere-equivalent ellipsoid (a=b=c=2)
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(2, 2, 2));
    math::Sphered sphere(2.0);

    // Fully above
    math::Planed planeAbove(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_NEAR(0.0, ellipsoid.VolumeBelow(planeAbove), 1e-10);

    // Fully below
    math::Planed planeBelow(math::Vector3d(0, 0, 1), 3.0);
    EXPECT_NEAR(ellipsoid.Volume(), ellipsoid.VolumeBelow(planeBelow), 1e-10);

    // Plane through center (half volume)
    math::Planed planeCenter(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2, ellipsoid.VolumeBelow(planeCenter),
                1e-10);

    // Should match sphere results
    EXPECT_NEAR(sphere.VolumeBelow(planeAbove),
                ellipsoid.VolumeBelow(planeAbove), 1e-10);
    EXPECT_NEAR(sphere.VolumeBelow(planeCenter),
                ellipsoid.VolumeBelow(planeCenter), 1e-10);
  }

  // Asymmetric ellipsoid (1, 2, 3)
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(1, 2, 3));

    // Plane through center along z-axis should give half volume
    math::Planed planeZ(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2, ellipsoid.VolumeBelow(planeZ), 1e-10);

    // Plane through center along x-axis should give half volume
    math::Planed planeX(math::Vector3d(1, 0, 0), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2, ellipsoid.VolumeBelow(planeX), 1e-10);

    // Plane through center along y-axis should give half volume
    math::Planed planeY(math::Vector3d(0, 1, 0), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2, ellipsoid.VolumeBelow(planeY), 1e-10);

    // Diagonal plane through center should give half volume
    math::Planed planeDiag(math::Vector3d(1, 1, 1).Normalized(), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2,
                ellipsoid.VolumeBelow(planeDiag), 1e-10);
  }

  // Complementary planes: V(n,d) + V(-n,-d) == totalVolume
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(1, 2, 3));
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed plane1(normal, offset);
    math::Planed plane2(-normal, -offset);

    EXPECT_NEAR(ellipsoid.Volume(),
                ellipsoid.VolumeBelow(plane1) + ellipsoid.VolumeBelow(plane2),
                1e-10);
  }

  // Non-unit normal
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(2, 2, 2));
    // Non-unit normal through center should still give half volume
    math::Planed plane(math::Vector3d(3, 4, 0), 0.0);
    EXPECT_NEAR(ellipsoid.Volume() / 2, ellipsoid.VolumeBelow(plane), 1e-10);
  }

  // Invalid ellipsoid (zero radii)
  {
    math::Ellipsoidd ellipsoid(math::Vector3d::Zero);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_DOUBLE_EQ(0.0, ellipsoid.VolumeBelow(plane));
  }
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, CenterOfVolumeBelow)
{
  // Sphere-equivalent ellipsoid
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(2, 2, 2));
    math::Sphered sphere(2.0);

    // Fully below: centroid at origin
    math::Planed planeBelow(math::Vector3d(0, 0, 1), 3.0);
    auto cov = ellipsoid.CenterOfVolumeBelow(planeBelow);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0, cov->X(), 1e-10);
    EXPECT_NEAR(0.0, cov->Y(), 1e-10);
    EXPECT_NEAR(0.0, cov->Z(), 1e-10);

    // Fully above: nullopt
    math::Planed planeAbove(math::Vector3d(0, 0, 1), -3.0);
    EXPECT_FALSE(ellipsoid.CenterOfVolumeBelow(planeAbove).has_value());

    // Should match sphere results for plane through center
    math::Planed planeCenter(math::Vector3d(0, 0, 1), 0.0);
    auto covEllipsoid = ellipsoid.CenterOfVolumeBelow(planeCenter);
    auto covSphere = sphere.CenterOfVolumeBelow(planeCenter);
    ASSERT_TRUE(covEllipsoid.has_value());
    ASSERT_TRUE(covSphere.has_value());
    EXPECT_NEAR(covSphere->X(), covEllipsoid->X(), 1e-10);
    EXPECT_NEAR(covSphere->Y(), covEllipsoid->Y(), 1e-10);
    EXPECT_NEAR(covSphere->Z(), covEllipsoid->Z(), 1e-10);
  }

  // Centroid direction: for plane through origin, n . centroid < 0
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(1, 2, 3));
    math::Vector3d normal(1, 1, 1);
    normal.Normalize();
    math::Planed plane(normal, 0.0);

    auto cov = ellipsoid.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_LT(normal.Dot(*cov), 0);
  }

  // Weighted sum property:
  // CoV_below * V_below + CoV_above * V_above == (0,0,0)
  {
    math::Ellipsoidd ellipsoid(math::Vector3d(1, 2, 3));
    math::Vector3d normal(1, 2, 3);
    normal.Normalize();
    double offset = 0.5;

    math::Planed plane(normal, offset);
    math::Planed flipped(-normal, -offset);

    auto vBelow = ellipsoid.VolumeBelow(plane);
    auto vAbove = ellipsoid.VolumeBelow(flipped);
    auto covBelow = ellipsoid.CenterOfVolumeBelow(plane);
    auto covAbove = ellipsoid.CenterOfVolumeBelow(flipped);

    ASSERT_TRUE(covBelow.has_value());
    ASSERT_TRUE(covAbove.has_value());

    auto weightedSum = (*covBelow) * vBelow + (*covAbove) * vAbove;
    EXPECT_NEAR(0.0, weightedSum.X(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Y(), 1e-10);
    EXPECT_NEAR(0.0, weightedSum.Z(), 1e-10);
  }

  // Invalid ellipsoid
  {
    math::Ellipsoidd ellipsoid(math::Vector3d::Zero);
    math::Planed plane(math::Vector3d(0, 0, 1), 0.0);
    EXPECT_FALSE(ellipsoid.CenterOfVolumeBelow(plane).has_value());
  }
}

//////////////////////////////////////////////////
TEST(EllipsoidTest, VolumeBelowFloat)
{
  // Use equal radii (sphere) for easy validation
  math::Ellipsoid<float> ellipsoid(math::Vector3<float>(2.0f, 2.0f, 2.0f));

  // Horizontal plane at z=0: half volume
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 0.0f);
    EXPECT_NEAR(ellipsoid.Volume() / 2.0f,
                ellipsoid.VolumeBelow(plane), 1e-3f);
  }

  // Fully below
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, 10.0f);
    EXPECT_NEAR(ellipsoid.Volume(), ellipsoid.VolumeBelow(plane), 1e-3f);
  }

  // Fully above
  {
    math::Plane<float> plane(math::Vector3<float>{0, 0, 1}, -10.0f);
    EXPECT_NEAR(0.0f, ellipsoid.VolumeBelow(plane), 1e-3f);
  }

  // CenterOfVolumeBelow with float
  {
    math::Plane<float> plane(math::Vector3<float>{0, 1, 0}, 0.0f);
    auto cov = ellipsoid.CenterOfVolumeBelow(plane);
    ASSERT_TRUE(cov.has_value());
    EXPECT_NEAR(0.0f, cov.value().X(), 1e-3f);
    EXPECT_NEAR(-0.75f, cov.value().Y(), 1e-3f);
    EXPECT_NEAR(0.0f, cov.value().Z(), 1e-3f);
  }
}
