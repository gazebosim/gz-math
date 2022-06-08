/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "ignition/math/MatrixX.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(MatrixXdTest, Construct)
{
  // Starts with zeroes
  MatrixX<int, 3, 1> mati31;
  for (int i = 0; i < 3; ++i)
  {
    EXPECT_EQ(0, mati31(i, 0)) << i;
  }

  // Initialize with values
  MatrixX<double, 1, 1> matd11(1.23);
  EXPECT_DOUBLE_EQ(1.23, matd11(0, 0));

  MatrixXd<6, 6> matd66(
      0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
      6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
      12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
      18.0, 19.0, 20.0, 21.0, 22.0, 23.0,
      24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
      30.0, 31.0, 32.0, 33.0, 34.0, 35.0);

  EXPECT_DOUBLE_EQ(matd66(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(matd66(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(matd66(0, 2), 2.0);
  EXPECT_DOUBLE_EQ(matd66(0, 3), 3.0);
  EXPECT_DOUBLE_EQ(matd66(0, 4), 4.0);
  EXPECT_DOUBLE_EQ(matd66(0, 5), 5.0);
  EXPECT_DOUBLE_EQ(matd66(1, 0), 6.0);
  EXPECT_DOUBLE_EQ(matd66(1, 1), 7.0);
  EXPECT_DOUBLE_EQ(matd66(1, 2), 8.0);
  EXPECT_DOUBLE_EQ(matd66(1, 3), 9.0);
  EXPECT_DOUBLE_EQ(matd66(1, 4), 10.0);
  EXPECT_DOUBLE_EQ(matd66(1, 5), 11.0);
  EXPECT_DOUBLE_EQ(matd66(2, 0), 12.0);
  EXPECT_DOUBLE_EQ(matd66(2, 1), 13.0);
  EXPECT_DOUBLE_EQ(matd66(2, 2), 14.0);
  EXPECT_DOUBLE_EQ(matd66(2, 3), 15.0);
  EXPECT_DOUBLE_EQ(matd66(2, 4), 16.0);
  EXPECT_DOUBLE_EQ(matd66(2, 5), 17.0);
  EXPECT_DOUBLE_EQ(matd66(3, 0), 18.0);
  EXPECT_DOUBLE_EQ(matd66(3, 1), 19.0);
  EXPECT_DOUBLE_EQ(matd66(3, 2), 20.0);
  EXPECT_DOUBLE_EQ(matd66(3, 3), 21.0);
  EXPECT_DOUBLE_EQ(matd66(3, 4), 22.0);
  EXPECT_DOUBLE_EQ(matd66(3, 5), 23.0);
  EXPECT_DOUBLE_EQ(matd66(4, 0), 24.0);
  EXPECT_DOUBLE_EQ(matd66(4, 1), 25.0);
  EXPECT_DOUBLE_EQ(matd66(4, 2), 26.0);
  EXPECT_DOUBLE_EQ(matd66(4, 3), 27.0);
  EXPECT_DOUBLE_EQ(matd66(4, 4), 28.0);
  EXPECT_DOUBLE_EQ(matd66(4, 5), 29.0);
  EXPECT_DOUBLE_EQ(matd66(5, 0), 30.0);
  EXPECT_DOUBLE_EQ(matd66(5, 1), 31.0);
  EXPECT_DOUBLE_EQ(matd66(5, 2), 32.0);
  EXPECT_DOUBLE_EQ(matd66(5, 3), 33.0);
  EXPECT_DOUBLE_EQ(matd66(5, 4), 34.0);
  EXPECT_DOUBLE_EQ(matd66(5, 5), 35.0);
  EXPECT_DOUBLE_EQ(matd66(100, 100), 35.0);

  // Copy constructor
  MatrixX<double, 1, 1> matd11Copy(matd11);
  EXPECT_DOUBLE_EQ(1.23, matd11Copy(0, 0));
  EXPECT_DOUBLE_EQ(1.23, matd11(0, 0));

  // Copy assignment
  MatrixX<double, 1, 1> matd11Copy2;
  matd11Copy2 = matd11;
  EXPECT_DOUBLE_EQ(1.23, matd11Copy2(0, 0));
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, CoverageExtra)
{
  // getting full destructor coverage
  auto p = new MatrixXi<3, 5>();
  EXPECT_NE(p, nullptr);
  delete p;
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, Addition)
{
  MatrixX<double, 2, 3> matd23A(
      0.1, 0.2, 0.3,
      0.4, 0.5, 0.6);

  MatrixX<double, 2, 3> matd23B(
      1.1, 1.2, 1.3,
      1.4, 1.5, 1.6);

  auto matSum = matd23A + matd23B;
  EXPECT_DOUBLE_EQ(matSum(0, 0), 1.2);
  EXPECT_DOUBLE_EQ(matSum(0, 1), 1.4);
  EXPECT_DOUBLE_EQ(matSum(0, 2), 1.6);
  EXPECT_DOUBLE_EQ(matSum(1, 0), 1.8);
  EXPECT_DOUBLE_EQ(matSum(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(matSum(1, 2), 2.2);
}

/////////////////////////////////////////////////
TEST(MatrixXTest, NoIndexException)
{
  MatrixXf<1, 2> mat;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      EXPECT_NO_THROW(mat(i, j));
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, OperatorStreamOut)
{
  MatrixX<float, 3, 1> mat(1.1f, 2.2f, 3.3f);
  std::ostringstream stream;
  stream << mat;
  EXPECT_EQ(stream.str(), "1.1 2.2 3.3");
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, OperatorStreamIn)
{
  MatrixX<double, 2, 3> matA;

  std::istringstream stream("1 2 3 4 5 6");
  stream >> matA;

  auto matB = MatrixX<double, 2, 3>(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  EXPECT_EQ(matA, matB);
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, NotEqual)
{
  {
    MatrixX<double, 1, 4> matrix1;
    MatrixX<double, 1, 4> matrix2;
    EXPECT_TRUE(matrix1 == matrix2);
    EXPECT_FALSE(matrix1 != matrix2);
    EXPECT_TRUE(matrix1.Equal(matrix2));
  }

  {
    MatrixX<double, 1, 4> matrix1(1.0, 2.0, 3.0, 4.0);
    MatrixX<double, 1, 4> matrix2(matrix1);

    EXPECT_TRUE(matrix1 == matrix1);
    EXPECT_FALSE(matrix1 != matrix1);
    EXPECT_TRUE(matrix1.Equal(matrix2));

    matrix2.SetElement(0u, 0u, 1.00001);
    EXPECT_TRUE(matrix1 != matrix2);
    EXPECT_FALSE(matrix1.Equal(matrix2));

    matrix2.SetElement(0u, 0u, 1.000001);
    EXPECT_FALSE(matrix1 != matrix2);
    EXPECT_TRUE(matrix1.Equal(matrix2));
  }
}

/////////////////////////////////////////////////
TEST(MatrixXdTest, Transpose)
{
  // Matrix and expected transpose
  MatrixX<double, 4, 2> mat42(
      -2.0, 4.0,
      0.1, 9.0,
      -7.0, 1.0,
      0.2, 3.0);

  MatrixX<double, 2, 4> mat24(
      -2.0, 0.1, -7.0, 0.2,
      4.0, 9.0, 1.0, 3.0);

  EXPECT_EQ(mat24.Transposed(), mat42);
  EXPECT_EQ(mat42.Transposed(), mat24);
}
