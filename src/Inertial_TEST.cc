/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gz/math/Inertial.hh"

using namespace gz;

/////////////////////////////////////////////////
/// \brief Compare quaternions, but allow rotations of PI about any axis.
void CompareModuloPi(const math::Quaterniond &_q1,
                     const math::Quaterniond &_q2,
                     const double _tol = 1e-6)
{
  const auto rotErrorEuler = (_q1.Inverse() * _q2).Euler();
  EXPECT_NEAR(sin(rotErrorEuler.X()), 0.0, _tol);
  EXPECT_NEAR(sin(rotErrorEuler.Y()), 0.0, _tol);
  EXPECT_NEAR(sin(rotErrorEuler.Z()), 0.0, _tol);
}

/////////////////////////////////////////////////
// Simple constructor, test default values
TEST(Inertiald_Test, Constructor)
{
  math::Inertiald inertial;
  EXPECT_EQ(inertial.Pose(), math::Pose3d::Zero);
  EXPECT_EQ(inertial.MassMatrix(), math::MassMatrix3d());
  EXPECT_EQ(inertial.Moi(), math::Matrix3d::Zero);
  EXPECT_FALSE(inertial.FluidAddedMass().has_value());
  EXPECT_EQ(inertial.BodyMatrix(), math::Matrix6d::Zero);
  EXPECT_EQ(inertial.SpatialMatrix(), math::Matrix6d::Zero);
  EXPECT_EQ(inertial.BodyMatrix(), inertial.SpatialMatrix());
}

/////////////////////////////////////////////////
// Constructor with default arguments
// Should match simple constructor and with copy constructor
TEST(Inertiald_Test, ConstructorDefaultValues)
{
  math::Inertiald inertial(math::MassMatrix3d(), math::Pose3d::Zero);
  EXPECT_EQ(inertial, math::Inertiald());
  EXPECT_EQ(inertial, math::Inertiald(inertial));
}

/////////////////////////////////////////////////
// Constructor with non-default arguments
TEST(Inertiald_Test, ConstructorNonDefaultValues)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, GZ_PI/6, 0, 0);
  math::Inertiald inertial(m, pose);

  // Should not match simple constructor
  EXPECT_NE(inertial, math::Inertiald());

  // Should match with copy constructor
  EXPECT_EQ(inertial, math::Inertiald(inertial));

  // Test accessors
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);
  EXPECT_TRUE(inertial.MassMatrix().IsPositive());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  // Test assignment operator
  math::Inertiald inertial2;
  EXPECT_NE(inertial, inertial2);
  inertial2 = inertial;
  EXPECT_EQ(inertial, inertial2);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, CoverageExtra)
{
  // getting full destructor coverage
  math::Inertiald *p = new math::Inertiald;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetMassMatrix)
{
  math::Inertiald inertial;
  math::MassMatrix3d m;

  // This will be true because the default mass of zero is considered valid
  EXPECT_TRUE(inertial.SetMassMatrix(m, 0));
  // Set the mass to a negative value, and SetMassMatrix should complain.
  m.SetMass(-1);
  EXPECT_FALSE(inertial.SetMassMatrix(m, 0));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, Setters)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, GZ_PI/6, 0, 0);
  math::Inertiald inertial;

  // Initially valid
  EXPECT_TRUE(inertial.SetPose(pose));

  // Valid once valid mass matrix is set
  EXPECT_TRUE(inertial.SetMassMatrix(m));

  // Verify values
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);

  // Invalid again if an invalid inertia is set
  math::MassMatrix3d mInvalid(-1, Ixxyyzz, Ixyxzyz);
  EXPECT_FALSE(inertial.SetMassMatrix(mInvalid));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, MOI_Diagonal)
{
  const double mass = 12.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0, 0, 0);
  const math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  // no rotation, expect MOI's to match
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, 0);
    math::Inertiald inertial(m, pose);
    EXPECT_EQ(inertial.Moi(), m.Moi());
  }

  // 90 deg rotation about X axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, GZ_PI_2, 0, 0);
    const math::Matrix3d expectedMOI(2, 0, 0, 0, 4, 0, 0, 0, 3);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.Moi(), m.Moi());
    EXPECT_EQ(inertial.Moi(), expectedMOI);
  }

  // 90 deg rotation about Y axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, GZ_PI_2, 0);
    const math::Matrix3d expectedMOI(4, 0, 0, 0, 3, 0, 0, 0, 2);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.Moi(), m.Moi());
    EXPECT_EQ(inertial.Moi(), expectedMOI);
  }

  // 90 deg rotation about Z axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, GZ_PI_2);
    const math::Matrix3d expectedMOI(3, 0, 0, 0, 2, 0, 0, 0, 4);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.Moi(), m.Moi());
    EXPECT_EQ(inertial.Moi(), expectedMOI);
  }

  // 45 deg rotation about Z axis, expect different MOI
  {
    const math::Pose3d pose(0, 0, 0, 0, 0, GZ_PI_4);
    const math::Matrix3d expectedMOI(2.5, -0.5, 0, -0.5, 2.5, 0, 0, 0, 4);
    math::Inertiald inertial(m, pose);
    EXPECT_NE(inertial.Moi(), m.Moi());
    EXPECT_EQ(inertial.Moi(), expectedMOI);

    // double check with a second MassMatrix3 instance
    // that has the same base frame MOI but no pose rotation
    math::MassMatrix3d m2;
    EXPECT_TRUE(m2.SetMass(mass));
    EXPECT_TRUE(m2.SetMoi(expectedMOI));
    EXPECT_EQ(inertial.Moi(), m2.Moi());
    // There are multiple correct rotations due to symmetry
    CompareModuloPi(m2.PrincipalAxesOffset(), pose.Rot());
  }
}

/////////////////////////////////////////////////
// Base frame MOI should be invariant
void SetRotation(const double _mass,
    const math::Vector3d &_ixxyyzz,
    const math::Vector3d &_ixyxzyz,
    const bool _unique = true)
{
  const math::MassMatrix3d m(_mass, _ixxyyzz, _ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  math::Pose3d pose(math::Vector3d::Zero, math::Quaterniond::Identity);
  const math::Inertiald inertialRef(m, pose);
  const auto moi = inertialRef.Moi();

  const std::vector<math::Quaterniond> rotations = {
    math::Quaterniond::Identity,
    math::Quaterniond(GZ_PI, 0, 0),
    math::Quaterniond(0, GZ_PI, 0),
    math::Quaterniond(0, 0, GZ_PI),
    math::Quaterniond(GZ_PI_2, 0, 0),
    math::Quaterniond(0, GZ_PI_2, 0),
    math::Quaterniond(0, 0, GZ_PI_2),
    math::Quaterniond(GZ_PI_4, 0, 0),
    math::Quaterniond(0, GZ_PI_4, 0),
    math::Quaterniond(0, 0, GZ_PI_4),
    math::Quaterniond(GZ_PI/6, 0, 0),
    math::Quaterniond(0, GZ_PI/6, 0),
    math::Quaterniond(0, 0, GZ_PI/6),
    math::Quaterniond(0.1, 0.2, 0.3),
    math::Quaterniond(-0.1, 0.2, -0.3),
    math::Quaterniond(0.4, 0.2, 0.5),
    math::Quaterniond(-0.1, 0.7, -0.7)};
  for (const auto &rot : rotations)
  {
    {
      auto inertial = inertialRef;

      const double tol  = -1e-6;
      EXPECT_TRUE(inertial.SetMassMatrixRotation(rot, tol));
      EXPECT_EQ(moi, inertial.Moi());
      if (_unique)
      {
        CompareModuloPi(rot, inertial.MassMatrix().PrincipalAxesOffset(tol));
      }

      EXPECT_TRUE(inertial.SetInertialRotation(rot));
      EXPECT_EQ(rot, inertial.Pose().Rot());
      EXPECT_EQ(moi, inertial.Moi());
    }

    {
      auto inertial = inertialRef;

      EXPECT_TRUE(inertial.SetInertialRotation(rot));
      EXPECT_EQ(rot, inertial.Pose().Rot());
      EXPECT_EQ(moi, inertial.Moi());

      const double tol = -1e-6;
      EXPECT_TRUE(inertial.SetMassMatrixRotation(rot, tol));
      EXPECT_EQ(moi, inertial.Moi());
      if (_unique)
      {
        CompareModuloPi(rot, inertial.MassMatrix().PrincipalAxesOffset(tol));
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationUniqueDiagonal)
{
  SetRotation(12, math::Vector3d(2, 3, 4), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(3, 2, 4), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(2, 4, 3), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(3, 4, 2), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(4, 2, 3), math::Vector3d::Zero);
  SetRotation(12, math::Vector3d(4, 3, 2), math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationUniqueNondiagonal)
{
  SetRotation(12, math::Vector3d(2, 3, 4), math::Vector3d(0.3, 0.2, 0.1));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationNonuniqueDiagonal)
{
  SetRotation(12, math::Vector3d(2, 2, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 2, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 3, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 2, 2), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(2, 3, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 2, 3), math::Vector3d::Zero, false);
  SetRotation(12, math::Vector3d(3, 3, 2), math::Vector3d::Zero, false);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SetRotationNonuniqueNondiagonal)
{
  SetRotation(12, math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0), false);
  SetRotation(12, math::Vector3d(4, 3, 4), math::Vector3d(0, -1, 0), false);
  SetRotation(12, math::Vector3d(3, 4, 4), math::Vector3d(0, 0, -1), false);
  SetRotation(12, math::Vector3d(4, 4, 5), math::Vector3d(-1, 0, 0), false);
  SetRotation(12, math::Vector3d(5, 4, 4), math::Vector3d(0, 0, -1), false);
  SetRotation(12, math::Vector3d(5.5, 4.125, 4.375),
             0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2), false);
  SetRotation(12, math::Vector3d(4.125, 5.5, 4.375),
                      0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0), false);
}

/////////////////////////////////////////////////
// test for diagonalizing MassMatrix
// verify MOI is conserved
// and that off-diagonal terms are zero
void Diagonalize(
    const double _mass,
    const math::Vector3d &_ixxyyzz,
    const math::Vector3d &_ixyxzyz)
{
  const math::MassMatrix3d m(_mass, _ixxyyzz, _ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  math::Pose3d pose(math::Vector3d::Zero, math::Quaterniond::Identity);
  math::Inertiald inertial(m, pose);
  const auto moi = inertial.Moi();

  EXPECT_TRUE(inertial.SetMassMatrixRotation(math::Quaterniond::Identity));
  EXPECT_EQ(moi, inertial.Moi());
  EXPECT_EQ(inertial.MassMatrix().OffDiagonalMoments(), math::Vector3d::Zero);

  // try again with negative tolerance
  EXPECT_TRUE(
    inertial.SetMassMatrixRotation(math::Quaterniond::Identity, -1e-6));
  EXPECT_EQ(moi, inertial.Moi());
  EXPECT_EQ(inertial.MassMatrix().OffDiagonalMoments(), math::Vector3d::Zero);
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, Diagonalize)
{
  Diagonalize(12, math::Vector3d(2, 3, 4), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 4), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 4, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 4, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 4), math::Vector3d(0.3, 0.2, 0.1));
  Diagonalize(12, math::Vector3d(2, 2, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(2, 3, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 2, 3), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(3, 3, 2), math::Vector3d::Zero);
  Diagonalize(12, math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0));
  Diagonalize(12, math::Vector3d(4, 3, 4), math::Vector3d(0, -1, 0));
  Diagonalize(12, math::Vector3d(3, 4, 4), math::Vector3d(0, 0, -1));
  Diagonalize(12, math::Vector3d(4, 4, 5), math::Vector3d(-1, 0, 0));
  Diagonalize(12, math::Vector3d(5, 4, 4), math::Vector3d(0, 0, -1));
  Diagonalize(12, math::Vector3d(5.5, 4.125, 4.375),
                               0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2));
  Diagonalize(12, math::Vector3d(4.125, 5.5, 4.375),
                      0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0));
}
/////////////////////////////////////////////////
TEST(Inertiald_Test, AdditionSubtraction)
{
  // Add two half-cubes together and
  // Subtract one half-cube from a full cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald cube(cubeMM3, math::Pose3d::Zero);
    math::MassMatrix3d half;
    const math::Vector3d half_size(0.5, 1, 1);
    EXPECT_TRUE(half.SetFromBox(0.5*mass, half_size));
    math::Inertiald left(half, math::Pose3d(-0.25, 0, 0, 0, 0, 0));
    math::Inertiald right(half, math::Pose3d(0.25, 0, 0, 0, 0, 0));
    EXPECT_EQ(cube, left + right);
    EXPECT_EQ(cube, right + left);
    EXPECT_EQ(right, cube - left);
    EXPECT_EQ(left, cube - right);

    // test += operator
    {
      math::Inertiald tmp = left;
      tmp += right;
      EXPECT_EQ(cube, tmp);
    }
    {
      math::Inertiald tmp = right;
      tmp += left;
      EXPECT_EQ(cube, tmp);
    }
    // test -= operator
    {
      math::Inertiald tmp = cube;
      tmp -= right;
      EXPECT_EQ(left, tmp);
    }
    {
      math::Inertiald tmp = cube;
      tmp -= left;
      EXPECT_EQ(right, tmp);
    }

    // Test EquivalentBox
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((left + right).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((right + left).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((cube - right).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(half_size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
    {
      math::Vector3d size2;
      math::Quaterniond rot2;
      EXPECT_TRUE((cube - left).MassMatrix().EquivalentBox(size2, rot2));
      EXPECT_EQ(half_size, size2);
      EXPECT_EQ(rot2, math::Quaterniond::Identity);
    }
  }

  // Add two rotated half-cubes together and
  // Subtract a rotated half-cube from rotated full-cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald cube(cubeMM3, math::Pose3d(0, 0, 0, GZ_PI_4, 0, 0));

    math::MassMatrix3d half;
    EXPECT_TRUE(half.SetFromBox(0.5*mass, math::Vector3d(0.5, 1, 1)));
    math::Inertiald left(half, math::Pose3d(-0.25, 0, 0, GZ_PI_4, 0, 0));
    math::Inertiald right(half, math::Pose3d(0.25, 0, 0, GZ_PI_4, 0, 0));

    // objects won't match exactly
    // since inertia matrices will all be in base frame
    // but mass, center of mass, and base-frame MOI should match
    // +operator
    EXPECT_NE(cube, left + right);
    EXPECT_NE(cube, right + left);
    EXPECT_DOUBLE_EQ(cubeMM3.Mass(), (left + right).MassMatrix().Mass());
    EXPECT_DOUBLE_EQ(cubeMM3.Mass(), (right + left).MassMatrix().Mass());
    EXPECT_EQ(cube.Pose().Pos(), (left + right).Pose().Pos());
    EXPECT_EQ(cube.Pose().Pos(), (right + left).Pose().Pos());
    EXPECT_EQ(cube.Moi(), (left + right).Moi());
    EXPECT_EQ(cube.Moi(), (right + left).Moi());
    // -operator
    EXPECT_NE(left, cube - right);
    EXPECT_NE(right, cube - left);
    EXPECT_DOUBLE_EQ(left.MassMatrix().Mass(),
        (cube - right).MassMatrix().Mass());
    EXPECT_DOUBLE_EQ(right.MassMatrix().Mass(),
        (cube - left).MassMatrix().Mass());
    EXPECT_EQ(left.Pose().Pos(), (cube - right).Pose().Pos());
    EXPECT_EQ(right.Pose().Pos(), (cube - left).Pose().Pos());
    EXPECT_EQ(left.Moi(), (cube - right).Moi());
    EXPECT_EQ(right.Moi(), (cube - left).Moi());
  }

  // Add eight cubes together into larger cube and
  // Subtract seven cubes from larger cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald sevenCubes =
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, 0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, 0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, 0.5, 0, 0, 0));
    const math::Inertiald lastCube =
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, 0.5, 0, 0, 0));
    const math::Inertiald addedCube = sevenCubes + lastCube;

    math::MassMatrix3d trueCubeMM3;
    EXPECT_TRUE(trueCubeMM3.SetFromBox(8*mass, 2*size));
    EXPECT_EQ(addedCube, math::Inertiald(trueCubeMM3, math::Pose3d::Zero));
    EXPECT_EQ(lastCube, addedCube - sevenCubes);
    EXPECT_EQ(sevenCubes, addedCube - lastCube);
  }

  // Add eight rotated cubes together into larger cube and
  // Subtract seven rotated cubes from larger cube
  {
    const double mass = 12.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    const math::Inertiald sevenCubes =
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, -0.5, 0, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, -0.5, GZ_PI_2, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, -0.5, 0, GZ_PI_2, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, -0.5, 0, 0, GZ_PI_2)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, 0.5, GZ_PI, 0, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(-0.5,  0.5, 0.5, 0, GZ_PI, 0)) +
      math::Inertiald(cubeMM3, math::Pose3d(0.5,  -0.5, 0.5, 0, 0, GZ_PI));
    const math::Inertiald lastCube =
      math::Inertiald(cubeMM3, math::Pose3d(0.5,   0.5, 0.5, 0, 0, 0));
    const math::Inertiald addedCube = sevenCubes + lastCube;

    math::MassMatrix3d trueCubeMM3;
    EXPECT_TRUE(trueCubeMM3.SetFromBox(8*mass, 2*size));
    EXPECT_EQ(addedCube, math::Inertiald(trueCubeMM3, math::Pose3d::Zero));
    EXPECT_EQ(lastCube, addedCube - sevenCubes);
    EXPECT_EQ(sevenCubes, addedCube - lastCube);
  }

  // Add two cubes with diagonal corners touching at one point
  //           ┌---------┐
  //           |         |
  //           |         |
  //           |         |
  //           |         |
  // ┌---------+---------┘
  // |         |
  // |         |
  // |         |
  // |         |
  // └---------┘
  {
    // properties of each cube to be added
    // side length: 1
    // mass: 6
    // diagonal moment of inertia values: 1
    // off-diagonal moment of inertia values: 0
    const double mass = 6.0;
    const math::Vector3d size(1, 1, 1);
    math::MassMatrix3d cubeMM3;
    EXPECT_TRUE(cubeMM3.SetFromBox(mass, size));
    EXPECT_EQ(
        math::Vector3d::One,
        cubeMM3.DiagonalMoments());
    EXPECT_EQ(
        math::Vector3d::Zero,
        cubeMM3.OffDiagonalMoments());

    const math::Inertiald cube1 =
        math::Inertiald(cubeMM3, math::Pose3d(-0.5, -0.5, -0.5, 0, 0, 0));
    const math::Inertiald cube2 =
        math::Inertiald(cubeMM3, math::Pose3d(0.5,  0.5, 0.5, 0, 0, 0));
    const math::Inertiald diagonalCubes = cube1 + cube2;

    // lumped mass = 6 + 6 = 12
    // lumped center of mass at (0, 0, 0)
    // lumped Moment of inertia:
    //   for each cube
    //   [ 1  0  0 ]       [ 0.5^2 + 0.5^2  -0.5*0.5            -0.5*0.5 ]
    //   [ 0  1  0 ] + 6 * [ -0.5*0.5       0.5^2 + 0.5^2       -0.5*0.5 ]
    //   [ 0  0  1 ]       [ -0.5*0.5       -0.5*0.5       0.5^2 + 0.5^2 ]
    //
    //   [ 1  0  0 ]       [  0.5   -0.25  -0.25 ]
    //   [ 0  1  0 ] + 6 * [ -0.25   0.5   -0.25 ]
    //   [ 0  0  1 ]       [ -0.25  -0.25   0.5  ]
    //
    //   [ 1  0  0 ]   [  3.0  -1.5  -1.5 ]
    //   [ 0  1  0 ] + [ -1.5   3.0  -1.5 ]
    //   [ 0  0  1 ]   [ -1.5  -1.5   3.0 ]
    //
    //   [  4.0  -1.5  -1.5 ]
    //   [ -1.5   4.0  -1.5 ]
    //   [ -1.5  -1.5   4.0 ]
    //
    // then double it to account for both cubes
    EXPECT_EQ(math::Pose3d::Zero, diagonalCubes.Pose());
    EXPECT_DOUBLE_EQ(mass * 2.0, diagonalCubes.MassMatrix().Mass());
    EXPECT_EQ(
        math::Vector3d(8, 8, 8),
        diagonalCubes.MassMatrix().DiagonalMoments());
    EXPECT_EQ(
        math::Vector3d(-3, -3, -3),
        diagonalCubes.MassMatrix().OffDiagonalMoments());

    // -operator
    EXPECT_EQ(cube1.Pose(), (diagonalCubes - cube2).Pose());
    EXPECT_EQ(cube2.Pose(), (diagonalCubes - cube1).Pose());
    EXPECT_DOUBLE_EQ(mass, (diagonalCubes - cube2).MassMatrix().Mass());
    EXPECT_DOUBLE_EQ(mass, (diagonalCubes - cube1).MassMatrix().Mass());
    EXPECT_EQ(
        cubeMM3.DiagonalMoments(),
        (diagonalCubes - cube2).MassMatrix().DiagonalMoments());
    EXPECT_EQ(
        cubeMM3.DiagonalMoments(),
        (diagonalCubes - cube1).MassMatrix().DiagonalMoments());
    EXPECT_EQ(
        cubeMM3.OffDiagonalMoments(),
        (diagonalCubes - cube2).MassMatrix().OffDiagonalMoments());
    EXPECT_EQ(
        cubeMM3.OffDiagonalMoments(),
        (diagonalCubes - cube1).MassMatrix().OffDiagonalMoments());
  }
}

/////////////////////////////////////////////////
// Addition operator has different behavior if mass is non-positive
TEST(Inertiald_Test, AdditionInvalid)
{
  // inertias all zero
  const math::MassMatrix3d m0(0.0, math::Vector3d::Zero, math::Vector3d::Zero);
  EXPECT_FALSE(m0.IsPositive());
  EXPECT_TRUE(m0.IsNearPositive());
  EXPECT_TRUE(m0.IsValid());

  // both inertials with zero mass
  {
    math::Inertiald left(m0, math::Pose3d(-1, 0, 0, 0, 0, 0));
    math::Inertiald right(m0, math::Pose3d(1, 0, 0, 0, 0, 0));

    // expect sum to equal left argument
    EXPECT_EQ(left, left + right);
    EXPECT_EQ(right, right + left);
    {
      math::Inertiald tmp = left;
      tmp += right;
      EXPECT_EQ(tmp, left);
    }
    {
      math::Inertiald tmp = right;
      tmp += left;
      EXPECT_EQ(tmp, right);
    }
  }

  // one inertial with zero inertias should not affect the sum
  {
    math::MassMatrix3d m(12.0,
      math::Vector3d(2, 3, 4),
      math::Vector3d(0.1, 0.2, 0.3));
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    math::Inertiald i(m, math::Pose3d(-1, 0, 0, 0, 0, 0));
    math::Inertiald i0(m0, math::Pose3d(1, 0, 0, 0, 0, 0));

    // expect i0 to not affect the sum
    EXPECT_EQ(i, i + i0);
    EXPECT_EQ(i, i0 + i);
    {
      math::Inertiald tmp = i;
      tmp += i0;
      EXPECT_EQ(tmp, i);
    }
    {
      math::Inertiald tmp = i0;
      tmp += i;
      EXPECT_EQ(tmp, i);
    }

    EXPECT_TRUE((i + i0).MassMatrix().IsPositive());
    EXPECT_TRUE((i0 + i).MassMatrix().IsPositive());
    EXPECT_TRUE((i + i0).MassMatrix().IsValid());
    EXPECT_TRUE((i0 + i).MassMatrix().IsValid());
  }
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, SubtractionInvalid)
{
  const double mass = 12.0;
  {
    math::MassMatrix3d m1, m2;
    EXPECT_TRUE(m1.SetFromBox(0.5*mass, math::Vector3d(0.5, 1, 1)));
    EXPECT_TRUE(m1.IsPositive());
    EXPECT_TRUE(m1.IsValid());
    EXPECT_TRUE(m2.SetFromBox(mass, math::Vector3d(0.5, 0.25, 0.25)));
    EXPECT_TRUE(m2.IsValid());
    EXPECT_TRUE(m2.IsPositive());

    // two inertials with i2 having higher mass than i1
    math::Inertiald i1(m1, math::Pose3d(-0.25, 0, 0, 0, 0, 0));
    math::Inertiald i2(m2, math::Pose3d(0.25, 0, 0, 0, 0, 0));

    // expect subtraction to equal left argument
    EXPECT_EQ(i1, i1 - i2);
    {
      math::Inertiald tmp = i1;
      tmp -= i2;
      EXPECT_EQ(tmp, i1);
    }
  }

  // one inertial with zero inertias should not affect the subtraction
  {
    const math::MassMatrix3d m1(mass,
                         math::Vector3d(2, 3, 4),
                         math::Vector3d(0.1, 0.2, 0.3));
    EXPECT_TRUE(m1.IsPositive());
    EXPECT_TRUE(m1.IsValid());

    const math::MassMatrix3d m2(0.0,
                                math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_FALSE(m2.IsPositive());
    EXPECT_TRUE(m2.IsNearPositive());
    EXPECT_TRUE(m2.IsValid());

    // i2 with zero inertia
    math::Inertiald i1(m1, math::Pose3d(-1, 0, 0, 0, 0, 0));
    math::Inertiald i2(m2, math::Pose3d(1, 0, 0, 0, 0, 0));

    // expect i2 to not affect the subtraction
    EXPECT_EQ(i1, i1 - i2);
    {
      math::Inertiald tmp = i1;
      tmp -= i2;
      EXPECT_EQ(tmp, i1);
    }

    EXPECT_TRUE((i1 - i2).MassMatrix().IsPositive());
    EXPECT_FALSE((i2 - i1).MassMatrix().IsPositive());
    EXPECT_TRUE((i1 - i2).MassMatrix().IsValid());
    EXPECT_TRUE((i2 - i1).MassMatrix().IsValid());
  }
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, BodyMatrix)
{
  math::MassMatrix3d massMatrix(100, {2.0, 3.0, 4.0}, {0.2, 0.3, 0.4});
  math::Pose3d com{7, 8, 9, 0, 0, GZ_PI * 0.5};

  math::Inertiald inertial(massMatrix, com);

  auto bodyMatrix = inertial.BodyMatrix();

  EXPECT_EQ(bodyMatrix, inertial.SpatialMatrix());

  // Mass diagonal
  EXPECT_EQ(math::Matrix3d(
      100,   0, 0,
      0,   100, 0,
      0,     0, 100),
      bodyMatrix.Submatrix(math::Matrix6d::TOP_LEFT));

  // CoM translational offset
  EXPECT_EQ(math::Matrix3d(
      0,        100 * 9,  -100 * 8,
      -100 * 9, 0,        100 * 7,
      100 * 8,  -100 * 7, 0),
      bodyMatrix.Submatrix(math::Matrix6d::TOP_RIGHT));

  // Transpose of TOP_RIGHT
  EXPECT_EQ(math::Matrix3d(
      0,        -100 * 9, 100 * 8,
      100 * 9,  0,        -100 * 7,
      -100 * 8, 100 * 7,  0),
      bodyMatrix.Submatrix(math::Matrix6d::BOTTOM_LEFT));
  EXPECT_EQ(bodyMatrix.Submatrix(math::Matrix6d::BOTTOM_LEFT).Transposed(),
      bodyMatrix.Submatrix(math::Matrix6d::TOP_RIGHT));

  // Moments of inertia with CoM rotational offset
  // 90 deg yaw:
  // * xx <- (-1)*(-1)*yy
  // * xy <- (-1)*xy
  // * xz <- (-1)*yz
  // * yy <- xx
  // * yz <- xz
  // * zz <- zz
  EXPECT_EQ(math::Matrix3d(
      3.0, -0.2, -0.4,
      -0.2, 2.0, 0.3,
      -0.4, 0.3, 4.0),
      bodyMatrix.Submatrix(math::Matrix6d::BOTTOM_RIGHT));
}

/////////////////////////////////////////////////
TEST(Inertiald_Test, FluidAddedMass)
{
  math::MassMatrix3d massMatrix(100, {1, 2, 3}, {4, 5, 6});
  math::Pose3d com{7, 8, 9, 0, 0, 0};
  math::Matrix6d addedMass{
      0.1, 0.2, 0.3, 0.4, 0.5, 0.6,
      0.2, 0.7, 0.8, 0.9, 1.0, 1.1,
      0.3, 0.8, 1.2, 1.3, 1.4, 1.5,
      0.4, 0.9, 1.3, 1.6, 1.7, 1.8,
      0.5, 1.0, 1.4, 1.7, 1.9, 2.0,
      0.6, 1.1, 1.5, 1.8, 2.0, 2.1};

  math::Inertiald inertial(massMatrix, com, addedMass);
  EXPECT_TRUE(inertial.FluidAddedMass().has_value());
  EXPECT_EQ(addedMass, inertial.FluidAddedMass());

  auto spatialMatrix = inertial.SpatialMatrix();

  EXPECT_EQ(math::Matrix3d(
      100 + 0.1, 0.2,       0.3,
      0.2,       100 + 0.7, 0.8,
      0.3,       0.8,       100 + 1.2),
      spatialMatrix.Submatrix(math::Matrix6d::TOP_LEFT));

  EXPECT_EQ(math::Matrix3d(
      0.4,           0.5 + 100 * 9, 0.6 - 100 * 8,
      0.9 - 100 * 9, 1.0,           1.1 + 100 * 7,
      1.3 + 100 * 8, 1.4 - 100 * 7, 1.5),
      spatialMatrix.Submatrix(math::Matrix6d::TOP_RIGHT));

  EXPECT_EQ(math::Matrix3d(
      0.4,           0.9 - 100 * 9, 1.3 + 100 * 8,
      0.5 + 100 * 9, 1.0,           1.4 - 100 * 7,
      0.6 - 100 * 8, 1.1 + 100 * 7, 1.5),
      spatialMatrix.Submatrix(math::Matrix6d::BOTTOM_LEFT));

  EXPECT_EQ(math::Matrix3d(
      1.6 + 1, 1.7 + 4, 1.8 + 5,
      1.7 + 4, 1.9 + 2, 2.0 + 6,
      1.8 + 5, 2.0 + 6, 2.1 +3),
      spatialMatrix.Submatrix(math::Matrix6d::BOTTOM_RIGHT));

  // Set new added mass
  math::Matrix6d notSymmetric;
  notSymmetric(1, 2) = 100;
  EXPECT_FALSE(inertial.SetFluidAddedMass(notSymmetric));

  math::Matrix6d newAddedMass{
      0.01, 0.02, 0.03, 0.04, 0.05, 0.06,
      0.02, 0.07, 0.08, 0.09, 1.00, 1.01,
      0.03, 0.08, 1.02, 1.03, 1.04, 1.05,
      0.04, 0.09, 1.03, 1.06, 1.07, 1.08,
      0.05, 1.00, 1.04, 1.07, 1.09, 2.00,
      0.06, 1.01, 1.05, 1.08, 2.00, 2.01};
  EXPECT_TRUE(inertial.SetFluidAddedMass(newAddedMass));
  EXPECT_EQ(newAddedMass, inertial.FluidAddedMass());

  auto newSpatialMatrix = inertial.SpatialMatrix();
  EXPECT_NE(newSpatialMatrix, spatialMatrix);

  EXPECT_EQ(math::Matrix3d(
      100 + 0.01, 0.02,       0.03,
      0.02,       100 + 0.07, 0.08,
      0.03,       0.08,       100 + 1.02),
      newSpatialMatrix.Submatrix(math::Matrix6d::TOP_LEFT));

  EXPECT_EQ(math::Matrix3d(
      0.04,           0.05 + 100 * 9, 0.06 - 100 * 8,
      0.09 - 100 * 9, 1.00,           1.01 + 100 * 7,
      1.03 + 100 * 8, 1.04 - 100 * 7, 1.05),
      newSpatialMatrix.Submatrix(math::Matrix6d::TOP_RIGHT));

  EXPECT_EQ(math::Matrix3d(
      0.04,           0.09 - 100 * 9, 1.03 + 100 * 8,
      0.05 + 100 * 9, 1.00,           1.04 - 100 * 7,
      0.06 - 100 * 8, 1.01 + 100 * 7, 1.05),
      newSpatialMatrix.Submatrix(math::Matrix6d::BOTTOM_LEFT));

  EXPECT_EQ(math::Matrix3d(
      1.06 + 1, 1.07 + 4, 1.08 + 5,
      1.07 + 4, 1.09 + 2, 2.00 + 6,
      1.08 + 5, 2.00 + 6, 2.01 +3),
      newSpatialMatrix.Submatrix(math::Matrix6d::BOTTOM_RIGHT));
}
