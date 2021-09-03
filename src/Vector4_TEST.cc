/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "ignition/math/Helpers.hh"
#include "ignition/math/Matrix4.hh"
#include "ignition/math/Vector4.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Vector4dTest, Construction)
{
  math::Vector4d vec(1, 0, 0, 0);

  // Copy constructor
  math::Vector4d vec2(vec);
  EXPECT_EQ(vec2, vec);

  // Copy operator
  math::Vector4d vec3;
  vec3 = vec;
  EXPECT_EQ(vec3, vec);

  // Move constructor
  math::Vector4d vec4(std::move(vec));
  EXPECT_EQ(vec4, vec2);
  vec = vec4;
  EXPECT_EQ(vec, vec2);

  // Move operator
  math::Vector4d vec5;
  vec5 = std::move(vec2);
  EXPECT_EQ(vec5, vec3);
  vec2 = vec5;
  EXPECT_EQ(vec2, vec3);

  // Inequality
  math::Vector4d vec6;
  EXPECT_NE(vec6, vec3);
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Vector4d)
{
  {
    math::Vector4d v;
    EXPECT_TRUE(math::equal(v.X(), 0.0));
    EXPECT_TRUE(math::equal(v.Y(), 0.0));
    EXPECT_TRUE(math::equal(v.Z(), 0.0));
    EXPECT_TRUE(math::equal(v.W(), 0.0));
  }

  math::Vector4d v1(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v1.X(), 1.0));
  EXPECT_TRUE(math::equal(v1.Y(), 2.0));
  EXPECT_TRUE(math::equal(v1.Z(), 3.0));
  EXPECT_TRUE(math::equal(v1.W(), 4.0));

  math::Vector4d v(v1);
  EXPECT_EQ(v, v1);

  // ::Distance
  EXPECT_TRUE(math::equal(v.Distance(
          math::Vector4d(0, 0, 0, 0)), 5.4772, 1e-3));
  EXPECT_TRUE(math::equal(v.Distance(1, 2, 3, 4), 0.0));

  // ::Length()
  v.Set(1, 2, 3, 4);
  EXPECT_TRUE(math::equal(v.Length(), 5.4772, 1e-3));

  // ::SquaredLength()
  EXPECT_TRUE(math::equal(v.SquaredLength(), 30.0));

  // ::Rounded
  v.Set(1.23, 2.34, 3.55, 8.49);
  EXPECT_TRUE(v.Rounded() == math::Vector4d(1, 2, 4, 8));

  // ::Round
  v.Round();
  EXPECT_TRUE(v == math::Vector4d(1, 2, 4, 8));

  // ::Correct
  v.Set(1, 1, std::nan("1"), 1);
  v.Correct();
  EXPECT_TRUE(v == math::Vector4d(1, 1, 0, 1));

  // ::Normalize
  v.Set(1, 2, 3, 4);
  v.Normalize();
  EXPECT_EQ(v, math::Vector4d(0.182574, 0.365148, 0.547723, 0.730297));

  // ::Normalized
  v.Set(1, 2, 3, 4);
  EXPECT_EQ(v.Normalized(),
            math::Vector4d(0.182574, 0.365148, 0.547723, 0.730297));

  // ::Set
  v.Set(2, 4, 6, 8);
  EXPECT_EQ(v, math::Vector4d(2, 4, 6, 8));

  // ::DotProd
  EXPECT_TRUE(math::equal(60.0, v.Dot(math::Vector4d(1, 2, 3, 4)), 1e-2));

  // ::AbsDotProd
  v1.Set(-1, -2, -3, -4);
  EXPECT_TRUE(math::equal(60.0, v.AbsDot(v1), 1e-2));

  // ::GetAbs
  EXPECT_TRUE(v1.Abs() == math::Vector4d(1, 2, 3, 4));
  EXPECT_TRUE(v.Abs() == math::Vector4d(2, 4, 6, 8));

  // ::operator= vector4
  v = v1;
  EXPECT_EQ(v, v1);

  // ::operator= double
  v = 1.2;
  EXPECT_EQ(v, math::Vector4d(1.2, 1.2, 1.2, 1.2));

  // ::operator+ vector4
  v = v + math::Vector4d(1, 2, 3, 4);
  EXPECT_EQ(v, math::Vector4d(2.2, 3.2, 4.2, 5.2));

  // ::operator+=
  v += v;
  EXPECT_EQ(v, math::Vector4d(4.4, 6.4, 8.4, 10.4));

  // ::operator- vector4
  v = v - math::Vector4d(1, 1, 1, 1);
  EXPECT_EQ(v, math::Vector4d(3.4, 5.4, 7.4, 9.4));

  // ::operator-= vector4
  v -= math::Vector4d(1, 1, 1, 1);
  EXPECT_EQ(v, math::Vector4d(2.4, 4.4, 6.4, 8.4));


  // ::operator/ vector4
  v = v / math::Vector4d(.1, .1, .1, .1);
  EXPECT_EQ(v, math::Vector4d(24, 44, 64, 84));

  // ::operator/ double
  v = v / 2.0;
  EXPECT_EQ(v, math::Vector4d(12, 22, 32, 42));

  // ::operator/= vector4
  v /= math::Vector4d(4, 4, 4, 4);
  EXPECT_EQ(v, math::Vector4d(3, 5.5, 8, 10.5));

  // ::operator/= double
  v /= .1;
  EXPECT_EQ(v, math::Vector4d(30, 55, 80, 105));

  // ::operator * matrix4
  v = v * math::Matrix4d(1, 2, 3, 4,
                        5, 6, 7, 8,
                        9, 10, 11, 12,
                        13, 14, 15, 16);
  EXPECT_EQ(v, math::Vector4d(2390, 2660, 2930, 3200));


  // ::operator * vector4
  v = v * math::Vector4d(.2, .3, .4, .5);
  EXPECT_EQ(v, math::Vector4d(478, 798, 1172, 1600));

  // ::operator *= vector4
  v *= math::Vector4d(2, 4, 6, 8);
  EXPECT_EQ(v, math::Vector4d(956, 3192, 7032, 12800));

  // ::operator * double
  v = v * 5.2;
  EXPECT_EQ(v, math::Vector4d(4971.2, 16598.4, 36566.4, 66560));

  // ::operator *= double
  v *= 10.0;
  EXPECT_EQ(v, math::Vector4d(49712, 1.65984e+05, 3.65664e+05, 6.656e+05));

  // ::operator != vector4
  EXPECT_NE(v, math::Vector4d());

  // ::operator < vector4
  v.Set(1, 2, 3, 4);
  EXPECT_TRUE(v < math::Vector4d(4, 3, 2, 1));

  // ::IsFinite
  EXPECT_TRUE(v.IsFinite());

  // ::operator[]
  v[0] = 1;
  v[1] = 2;
  v[2] = 3;
  v[3] = 4;
  EXPECT_DOUBLE_EQ(v[0], 1);
  EXPECT_DOUBLE_EQ(v[1], 2);
  EXPECT_DOUBLE_EQ(v[2], 3);
  EXPECT_DOUBLE_EQ(v[3], 4);
}

/////////////////////////////////////////////////
TEST(Vector4dTest, NoException)
{
  math::Vector4d v(1, 2, 3, 4);
  EXPECT_NO_THROW(math::equal(v[0], 1.0));
  EXPECT_NO_THROW(math::equal(v[1], 2.0));
  EXPECT_NO_THROW(math::equal(v[2], 3.0));
  EXPECT_NO_THROW(math::equal(v[3], 4.0));

  EXPECT_NO_THROW(math::equal(v[4], 5.0));
  EXPECT_DOUBLE_EQ(v[4], 4.0);
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Max)
{
  math::Vector4d vec1(0.1, 0.2, 0.3, 0.2);
  math::Vector4d vec2(0.2, 0.3, 0.4, 0.3);
  math::Vector4d vec3(0.1, 0.2, 0.3, 0.4);

  EXPECT_DOUBLE_EQ(vec1.Max(), 0.3);

  vec1.Max(vec2);
  EXPECT_EQ(vec1, math::Vector4d(0.2, 0.3, 0.4, 0.3));

  vec1.Max(vec3);
  EXPECT_EQ(vec1, math::Vector4d(0.2, 0.3, 0.4, 0.4));
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Min)
{
  math::Vector4d vec1(0.1, 0.2, 0.3, 0.4);
  math::Vector4d vec2(0.2, 0.3, 0.4, 0.3);
  math::Vector4d vec3(0.05, 0.1, 0.2, 0.2);

  EXPECT_DOUBLE_EQ(vec1.Min(), 0.1);

  vec1.Min(vec2);
  EXPECT_EQ(vec1, math::Vector4d(0.1, 0.2, 0.3, 0.3));

  vec1.Min(vec3);
  EXPECT_EQ(vec1, math::Vector4d(0.05, 0.1, 0.2, 0.2));
}

/////////////////////////////////////////////////
// Test Equal function with specified tolerance
TEST(Vector4dTest, EqualTolerance)
{
  EXPECT_FALSE(math::Vector4d::Zero.Equal(math::Vector4d::One, 1e-6));
  EXPECT_FALSE(math::Vector4d::Zero.Equal(math::Vector4d::One, 1e-3));
  EXPECT_FALSE(math::Vector4d::Zero.Equal(math::Vector4d::One, 1e-1));
  EXPECT_TRUE(math::Vector4d::Zero.Equal(math::Vector4d::One, 1));
  EXPECT_TRUE(math::Vector4d::Zero.Equal(math::Vector4d::One, 1.1));
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Distance)
{
  math::Vector4d vec1(0, 0, 0, 0);
  math::Vector4d vec2(1, 2, 3, 4);

  double dist = vec1.Distance(vec2);
  EXPECT_NEAR(dist, 5.477225, 1e-6);

  double dist2 = vec1.Distance(1, 2, 3, 4);
  EXPECT_DOUBLE_EQ(dist, dist2);

  math::Vector4i vecInt(0, 0, 0, 0);
  int distInt = vecInt.Distance({1, 1, 1, 1});
  EXPECT_EQ(distInt, 2);
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Sum)
{
  math::Vector4d vec1(1.5, 2.5, 3.5, -4.5);

  EXPECT_TRUE(math::equal(math::Vector4d::Zero.Sum(), 0.0, 1e-6));
  EXPECT_TRUE(math::equal(math::Vector4d::One.Sum(), 4.0, 1e-6));
  EXPECT_TRUE(math::equal(vec1.Sum(), 3.0, 1e-6));
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Add)
{
  math::Vector4d vec1(0.1, 0.2, 0.4, 0.8);
  math::Vector4d vec2(1.1, 2.2, 3.4, 4.8);

  math::Vector4d vec3 = vec1;
  vec3 += vec2;

  EXPECT_EQ(vec1 + vec2, math::Vector4d(1.2, 2.4, 3.8, 5.6));
  EXPECT_EQ(vec3, math::Vector4d(1.2, 2.4, 3.8, 5.6));

  // Add zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 + vec1, vec1);
    EXPECT_EQ(vec1 + 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector4d::Zero + vec1, vec1);
    EXPECT_EQ(vec1 + math::Vector4d::Zero, vec1);

    // Addition assignment
    math::Vector4d vec4(vec1);
    vec4 += 0;
    EXPECT_EQ(vec4, vec1);
    vec4 += math::Vector4d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Add non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 + vec1, math::Vector4d(2.6, 2.7, 2.9, 3.3));
    EXPECT_EQ(vec1 + 2.5, math::Vector4d(2.6, 2.7, 2.9, 3.3));

    math::Vector4d vec4(vec1);
    vec4 += 2.5;
    EXPECT_EQ(vec4, math::Vector4d(2.6, 2.7, 2.9, 3.3));
  }
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Sub)
{
  math::Vector4d vec1(0.1, 0.2, 0.4, 0.8);
  math::Vector4d vec2(1.1, 2.2, 3.4, 4.8);

  math::Vector4d vec3 = vec2;
  vec3 -= vec1;

  EXPECT_EQ(vec2 - vec1, math::Vector4d(1.0, 2.0, 3.0, 4.0));
  EXPECT_EQ(vec3, math::Vector4d(1.0, 2.0, 3.0, 4.0));

  // Subtraction with zeros
  {
    // Scalar left and right
    EXPECT_EQ(0 - vec1, -vec1);
    EXPECT_EQ(vec1 - 0, vec1);

    // Vector left and right
    EXPECT_EQ(math::Vector4d::Zero - vec1, -vec1);
    EXPECT_EQ(vec1 - math::Vector4d::Zero, vec1);

    // Subtraction assignment
    math::Vector4d vec4(vec1);
    vec4 -= 0;
    EXPECT_EQ(vec4, vec1);
    vec4 -= math::Vector4d::Zero;
    EXPECT_EQ(vec4, vec1);
  }

  // Subtract non-trivial scalar values left and right
  {
    EXPECT_EQ(2.5 - vec1, math::Vector4d(2.4, 2.3, 2.1, 1.7));
    EXPECT_EQ(vec1 - 2.5, -math::Vector4d(2.4, 2.3, 2.1, 1.7));

    math::Vector4d vec4(vec1);
    vec4 -= 2.5;
    EXPECT_EQ(vec4, -math::Vector4d(2.4, 2.3, 2.1, 1.7));
  }
}

/////////////////////////////////////////////////
TEST(Vector4dTest, OperatorStreamOut)
{
  math::Vector4d v(0.1, 1.2, 2.3, 0.0);
  std::ostringstream stream;
  stream << v;
  EXPECT_EQ(stream.str(), "0.1 1.2 2.3 0");
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Multiply)
{
  math::Vector4d v(0.1, -4.2, 11.1, 8.4);

  // Multiply by zero
  {
    // Scalar left and right
    EXPECT_EQ(0 * v, math::Vector4d::Zero);
    EXPECT_EQ(v * 0, math::Vector4d::Zero);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector4d::Zero, math::Vector4d::Zero);
  }

  // Multiply by one
  {
    // Scalar left and right
    EXPECT_EQ(1 * v, v);
    EXPECT_EQ(v * 1, v);

    // Element-wise vector multiplication
    EXPECT_EQ(v * math::Vector4d::One, v);
  }

  // Multiply by non-trivial scalar value
  {
    const double scalar = 2.5;
    math::Vector4d expect(0.25, -10.5, 27.75, 21.0);
    EXPECT_EQ(scalar * v, expect);
    EXPECT_EQ(v * scalar, expect);
  }

  // Multiply by itself element-wise
  EXPECT_EQ(v*v, math::Vector4d(0.01, 17.64, 123.21, 70.56));
}

/////////////////////////////////////////////////
TEST(Vector4dTest, Length)
{
  // Zero vector
  EXPECT_DOUBLE_EQ(math::Vector4d::Zero.Length(), 0.0);
  EXPECT_DOUBLE_EQ(math::Vector4d::Zero.SquaredLength(), 0.0);

  // One vector
  EXPECT_DOUBLE_EQ(math::Vector4d::One.Length(), 2.0);
  EXPECT_DOUBLE_EQ(math::Vector4d::One.SquaredLength(), 4.0);

  // Arbitrary vector
  math::Vector4d v(0.1, -4.2, 2.5, 1.0);
  EXPECT_NEAR(v.Length(), 4.98998997995, 1e-10);
  EXPECT_DOUBLE_EQ(v.SquaredLength(), 24.9);

  // Integer vector
  EXPECT_EQ(math::Vector4i::One.Length(), 2);
  EXPECT_EQ(math::Vector4i::One.SquaredLength(), 4);
}

/////////////////////////////////////////////////
TEST(Vector4Test, NaN)
{
  auto nanVec = math::Vector4d::NaN;
  EXPECT_FALSE(nanVec.IsFinite());
  EXPECT_TRUE(math::isnan(nanVec.X()));
  EXPECT_TRUE(math::isnan(nanVec.Y()));
  EXPECT_TRUE(math::isnan(nanVec.Z()));
  EXPECT_TRUE(math::isnan(nanVec.W()));

  nanVec.Correct();
  EXPECT_EQ(math::Vector4d::Zero, nanVec);
  EXPECT_TRUE(nanVec.IsFinite());

  auto nanVecF = math::Vector4f::NaN;
  EXPECT_FALSE(nanVecF.IsFinite());
  EXPECT_TRUE(math::isnan(nanVecF.X()));
  EXPECT_TRUE(math::isnan(nanVecF.Y()));
  EXPECT_TRUE(math::isnan(nanVecF.Z()));
  EXPECT_TRUE(math::isnan(nanVecF.W()));

  nanVecF.Correct();
  EXPECT_EQ(math::Vector4f::Zero, nanVecF);
  EXPECT_TRUE(nanVecF.IsFinite());
}
