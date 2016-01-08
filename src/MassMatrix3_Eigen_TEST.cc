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

#ifndef _USE_MATH_DEFINES
# define _USE_MATH_DEFINES
#endif
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include <cmath>

#include "ignition/math/Helpers.hh"
#include "ignition/math/MassMatrix3.hh"

using namespace ignition;

const int repeatCount = 1000000;
// const math::Vector3d Ixxyyzz(4, 5, 6);
// const math::Vector3d Ixyxzyz(0.1, 0.2, 0.3);
// example values with large condition number
const math::Vector3d Ixxyyzz(1e40, 1e20, 1);
const math::Vector3d Ixyxzyz(1e19, 1e21, 1e9);
const double relativeTolerance = 1e-30;

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, ValuesEigenIterative)
{
  using namespace Eigen;
  SelfAdjointEigenSolver<Matrix3d> es;
  Matrix3d X = Matrix3d();
  X << Ixxyyzz[0], Ixyxzyz[0], Ixyxzyz[1],
       Ixyxzyz[0], Ixxyyzz[1], Ixyxzyz[2],
       Ixyxzyz[1], Ixyxzyz[2], Ixxyyzz[2];
  for (int i = 0; i < repeatCount; ++i)
    es.compute(X, EigenvaluesOnly);
  std::cout << "Eigenvalues are: " << es.eigenvalues().transpose()
            << std::endl;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, ValuesEigenDirect)
{
  using namespace Eigen;
  SelfAdjointEigenSolver<Matrix3d> es;
  Matrix3d X = Matrix3d();
  X << Ixxyyzz[0], Ixyxzyz[0], Ixyxzyz[1],
       Ixyxzyz[0], Ixxyyzz[1], Ixyxzyz[2],
       Ixyxzyz[1], Ixyxzyz[2], Ixxyyzz[2];
  for (int i = 0; i < repeatCount; ++i)
    es.computeDirect(X, EigenvaluesOnly);
  std::cout << "Eigenvalues are: " << es.eigenvalues().transpose()
            << std::endl;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, ValuesIgnition)
{
  const double mass = 1.0;
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  for (int i = 0; i < repeatCount; ++i)
    m.PrincipalMoments(relativeTolerance);
  std::cout << "Eigenvalues are: " << m.PrincipalMoments(relativeTolerance)
            << std::endl;
}


/////////////////////////////////////////////////
TEST(MassMatrix3dTest, VectorsEigenIterative)
{
  using namespace Eigen;
  SelfAdjointEigenSolver<Matrix3d> es;
  Matrix3d X = Matrix3d();
  X << Ixxyyzz[0], Ixyxzyz[0], Ixyxzyz[1],
       Ixyxzyz[0], Ixxyyzz[1], Ixyxzyz[2],
       Ixyxzyz[1], Ixyxzyz[2], Ixxyyzz[2];
  for (int i = 0; i < repeatCount; ++i)
    es.compute(X, ComputeEigenvectors);
  std::cout << "Eigenvalues are: " << es.eigenvalues().transpose()
            << std::endl;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, VectorsEigenDirect)
{
  using namespace Eigen;
  SelfAdjointEigenSolver<Matrix3d> es;
  Matrix3d X = Matrix3d();
  X << Ixxyyzz[0], Ixyxzyz[0], Ixyxzyz[1],
       Ixyxzyz[0], Ixxyyzz[1], Ixyxzyz[2],
       Ixyxzyz[1], Ixyxzyz[2], Ixxyyzz[2];
  for (int i = 0; i < repeatCount; ++i)
    es.computeDirect(X, ComputeEigenvectors);
  std::cout << "Eigenvalues are: " << es.eigenvalues().transpose()
            << std::endl;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, VectorsIgnition)
{
  const double mass = 1.0;
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  for (int i = 0; i < repeatCount; ++i)
    m.PrincipalAxesOffset(relativeTolerance);
  std::cout << "Eigenvalues are: " << m.PrincipalMoments(relativeTolerance)
            << std::endl;
}

