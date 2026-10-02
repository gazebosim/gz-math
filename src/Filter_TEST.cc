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

#include "gz/math/Filter.hh"

using namespace gz;

/////////////////////////////////////////////////
TEST(FilterTest, OnePole)
{
  math::OnePole<double> filterA;
  EXPECT_DOUBLE_EQ(filterA.Process(0.2), 0.0);

  filterA.Fc(0.6, 1.4);
  EXPECT_DOUBLE_EQ(filterA.Process(2.5), 2.3307710879153634);

  math::OnePole<double> filterB(0.1, 0.2);
  EXPECT_DOUBLE_EQ(filterB.Process(0.5), 0.47839304086811385);

  filterB.Set(5.4);
  EXPECT_DOUBLE_EQ(filterB.Value(), 5.4);
}

/////////////////////////////////////////////////
TEST(FilterTest, OnePoleQuaternion)
{
  math::OnePoleQuaternion filterA;
  EXPECT_EQ(filterA.Value(), math::Quaterniond(1, 0, 0, 0));

  math::OnePoleQuaternion filterB(0.4, 1.4);
  EXPECT_EQ(filterB.Value(), math::Quaterniond(1, 0, 0, 0));

  EXPECT_EQ(filterA.Process(math::Quaterniond(0.1, 0.2, 0.3)),
            math::Quaterniond(1, 0, 0, 0));

  EXPECT_EQ(filterB.Process(math::Quaterniond(0.1, 0.2, 0.3)),
            math::Quaterniond(0.98841, 0.0286272, 0.0885614, 0.119929));

  filterA.Set(math::Quaterniond(0.707, 0, 0.707, 0));
  EXPECT_EQ(filterA.Value(), math::Quaterniond(0.707, 0, 0.707, 0));
}

/////////////////////////////////////////////////
TEST(FilterTest, OnePoleVector3)
{
  math::OnePoleVector3 filterA;
  EXPECT_EQ(filterA.Value(), math::Vector3d(0, 0, 0));

  math::OnePoleVector3 filterB(1.2, 3.4);
  EXPECT_EQ(filterB.Value(), math::Vector3d(0, 0, 0));

  EXPECT_EQ(filterA.Process(math::Vector3d(0.1, 0.2, 0.3)),
            math::Vector3d(0, 0, 0));

  EXPECT_EQ(filterB.Process(math::Vector3d(0.1, 0.2, 0.3)),
            math::Vector3d(0.089113, 0.178226, 0.267339));

  filterA.Set(math::Vector3d(1, 2, 3));
  EXPECT_EQ(filterA.Value(), math::Vector3d(1, 2, 3));
}

/////////////////////////////////////////////////
TEST(FilterTest, Biquad)
{
  math::BiQuad<double> filterA;
  EXPECT_NEAR(filterA.Value(), 0.0, 1e-10);
  EXPECT_NEAR(filterA.Process(1.1), 0.0, 1e-10);

  filterA.Fc(0.3, 1.4);
  EXPECT_DOUBLE_EQ(filterA.Process(1.2), 0.66924691484768517);

  filterA.Fc(0.3, 1.4, 0.1);
  EXPECT_DOUBLE_EQ(filterA.Process(10.25), 0.96057152402651302);
  // Multi-step Process calls to exercise state update
  EXPECT_DOUBLE_EQ(filterA.Process(5.0), 2.2809792005709335);
  EXPECT_DOUBLE_EQ(filterA.Process(2.0), 2.2786852100645718);

  math::BiQuad<double> filterB(4.3, 10.6);
  EXPECT_NEAR(filterB.Value(), 0.0, 1e-10);
  EXPECT_DOUBLE_EQ(filterB.Process(0.1234),  0.072418159950486546);

  filterB.Set(4.5);
  EXPECT_DOUBLE_EQ(filterB.Value(), 4.5);
}

/////////////////////////////////////////////////
TEST(FilterTest, BiquadVector3)
{
  math::BiQuadVector3 filterA;
  EXPECT_EQ(filterA.Value(), math::Vector3d(0, 0, 0));
  EXPECT_EQ(filterA.Process(math::Vector3d(1.1, 2.3, 3.4)),
            math::Vector3d(0, 0, 0));

  math::BiQuadVector3 filterB(6.5, 22.4);
  EXPECT_EQ(filterB.Value(), math::Vector3d(0, 0, 0));
  EXPECT_EQ(filterB.Process(math::Vector3d(0.1, 20.3, 33.45)),
            math::Vector3d(0.031748, 6.44475, 10.6196));

  filterA.Set(math::Vector3d(4, 5, 6));
  EXPECT_EQ(filterA.Value(), math::Vector3d(4, 5, 6));

  filterA.Fc(0.5, 2.0);
  EXPECT_EQ(filterA.Process(math::Vector3d(1.0, 2.0, 3.0)),
            math::Vector3d(3.25, 4.25, 5.25));

  filterA.Fc(0.5, 2.0, 0.2);
  EXPECT_EQ(filterA.Process(math::Vector3d(1.0, 2.0, 3.0)),
            math::Vector3d(19.0 / 7.0, 26.0 / 7.0, 33.0 / 7.0));
}

/////////////////////////////////////////////////
TEST(FilterTest, TemplateTypes)
{
  // OnePole template instantiations
  math::OnePole<float> filterF(0.6f, 1.4f);
  EXPECT_FLOAT_EQ(filterF.Process(2.5f), 2.330771f);
  filterF.Set(1.5f);
  EXPECT_FLOAT_EQ(filterF.Value(), 1.5f);

  math::OnePole<int> filterI(1, 2);
  filterI.Set(10);
  EXPECT_EQ(filterI.Value(), 10);

  // BiQuad template instantiations
  math::BiQuad<float> biquadF(4.3f, 10.6f);
  EXPECT_NEAR(biquadF.Value(), 0.0f, 1e-6f);
  biquadF.Set(3.5f);
  EXPECT_FLOAT_EQ(biquadF.Value(), 3.5f);

  math::BiQuad<int> biquadI(1, 2);
  biquadI.Set(20);
  EXPECT_EQ(biquadI.Value(), 20);
}

