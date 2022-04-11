/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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


#include <ignition/math/VolumetricGridLookupField.hh>

#include <gtest/gtest.h>
using namespace ignition;
using namespace math;


TEST(VolumetricGridLookupField, CheckInterpolationExact)
{
  // This tests query performance of 54000 points in a 3D grid.
  std::vector<Vector3d> cloud;
  const double stride_x = 1, stride_y = 5, stride_z = 10;
  for(double x = 0; x < 300; x += stride_x)
  {
    for(double y = 0; y < 300; y += stride_y)
    {
      for(double z = 0; z < 300; z += stride_z)
      {
        cloud.emplace_back(x, y, z);
      }
    }
  }
  VolumetricGridLookupField<double> scalarIndex(cloud);

  for(std::size_t i = 0; i < cloud.size(); ++i)
  {
    auto val = scalarIndex.GetInterpolators(cloud[i]);
    ASSERT_EQ(val.size(), 1);
    ASSERT_EQ(val[0], i);
  }
}

TEST(VolumetricGridLookupField, CheckInterpolationBoxEightPoints)
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

  VolumetricGridLookupField<double> scalarIndex(cloud);

  {
    // Inside, return 8 points
    auto pos =  Vector3d(0.5, 0.5, 0.5);
    auto indices = scalarIndex.GetInterpolators(pos);
    ASSERT_EQ(indices.size(), 8UL);
  }

  {
    // Outside, return 0 points
    auto pos =  Vector3d(-0.5, -0.5, -0.5);
    auto indices = scalarIndex.GetInterpolators(pos);
    ASSERT_EQ(indices.size(), 0UL);
  }

  // On plane, rerutn 4 points
  {
    auto pos =  Vector3d(0.5, 0.5, 0);
    auto indices = scalarIndex.GetInterpolators(pos);
    ASSERT_EQ(indices.size(), 4UL);
  }
  // On edge, return 2 points
  {
    auto pos =  Vector3d(0.5, 0, 0);
    auto indices = scalarIndex.GetInterpolators(pos);
    ASSERT_EQ(indices.size(), 2UL);
  }
}