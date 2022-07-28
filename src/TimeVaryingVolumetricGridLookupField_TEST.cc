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
#include <gz/math/TimeVaryingVolumetricGridLookupField.hh>
#include <gtest/gtest.h>
using namespace gz;
using namespace math;
/////////////////////////////////////////////////
TEST(TimeVaryingVolumetricLookupFieldTest, TestConstruction)
{
  std::vector<Vector3d> cloud;
  cloud.emplace_back(0, 0, 0);
  cloud.emplace_back(0, 0, 1);
  cloud.emplace_back(0, 1, 0);
  cloud.emplace_back(0, 1, 1);
  cloud.emplace_back(1, 0, 0);
  cloud.emplace_back(1, 0, 1);
  cloud.emplace_back(1, 1, 0);
  cloud.emplace_back(1, 1, 1);

  std::vector<double> valuesTime0{0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<double> valuesTime1{1, 1, 1, 1, 1, 1, 1, 1};

  VolumetricGridLookupField<double> scalarIndex(cloud);
  TimeVaryingVolumetricGridLookupField<
    double, double, InMemorySession<double, double>> timeVaryingField;

  timeVaryingField.AddVolumetricGridField(0, scalarIndex);
  timeVaryingField.AddVolumetricGridField(1, scalarIndex);

  {
    // Get data at T=0
    auto session = timeVaryingField.CreateSession();
    auto points = timeVaryingField.LookUp(session, Vector3d{0.5, 0.5, 0.5});
    auto res = timeVaryingField.EstimateQuadrilinear<double>(
      session,
      points,
      valuesTime0,
      valuesTime1,
      Vector3d{0.5, 0.5, 0.5}, -1);
    ASSERT_EQ(res, 0);

    // Step session
    auto next_session = timeVaryingField.StepTo(session, 0.5);

    ASSERT_TRUE(next_session.has_value());

    points = timeVaryingField.LookUp(
      next_session.value(), Vector3d{0.5, 0.5, 0.5});
    res = timeVaryingField.EstimateQuadrilinear<double>(
      session,
      points,
      valuesTime0,
      valuesTime1,
      Vector3d{0.5, 0.5, 0.5}, -1);
    ASSERT_EQ(res, 0.5);

    // Step session
    next_session = timeVaryingField.StepTo(next_session.value(), 1);
    ASSERT_TRUE(next_session.has_value());

    points = timeVaryingField.LookUp(next_session.value(),
      Vector3d{0.5, 0.5, 0.5});
    res = timeVaryingField.EstimateQuadrilinear<double>(
      session,
      points,
      valuesTime0,
      valuesTime1,
      Vector3d{0.5, 0.5, 0.5}, -1);
    ASSERT_EQ(res, 1);
  }
}
