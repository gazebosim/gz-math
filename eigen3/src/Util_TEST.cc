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

#include <gtest/gtest.h>

#include <ignition/math/eigen3/Util.hh>

using namespace ignition;

/////////////////////////////////////////////////
/// \brief Test the oriented box converted from a set of vertices
TEST(EigenUtil, meshToOrientedBox)
{
  std::vector<math::Vector3d> mesh;

  mesh.push_back(math::Vector3d(1, 0, 0.5));
  mesh.push_back(math::Vector3d(2, 0.1, 0.4));
  mesh.push_back(math::Vector3d(2, 1, 3));
  mesh.push_back(math::Vector3d(1.6, 0.3, 0.1));
  mesh.push_back(math::Vector3d(1.5, 0.5, 1));
  mesh.push_back(math::Vector3d(1.4, 1, 3));
  mesh.push_back(math::Vector3d(1, 0.4, 0.7));
  mesh.push_back(math::Vector3d(0.9, 1.3, 0));
  mesh.push_back(math::Vector3d(0.6, 4, 2));
  mesh.push_back(math::Vector3d(0, 3, 3));
  mesh.push_back(math::Vector3d(-1, -2, 4));
  mesh.push_back(math::Vector3d(-2, -2, 0.6));

  math::OrientedBoxd box = math::eigen3::meshToOrientedBox(mesh);

  auto position = box.Pose().Pos();
  auto rotation = box.Pose().Rot();
  auto size = box.Size();

  double error = 0.1;

  EXPECT_NEAR(size.X(), 3.09, error);
  EXPECT_NEAR(size.Y(), 4.39, error);
  EXPECT_NEAR(size.Z(), 6.63, error);

  EXPECT_NEAR(position.X(), 0.38, error);
  EXPECT_NEAR(position.Y(), 0.47, error);
  EXPECT_NEAR(position.Z(), 1.96, error);

  EXPECT_NEAR(rotation.Roll(), -1.66, error);
  EXPECT_NEAR(rotation.Pitch(), 0.4, error);
  EXPECT_NEAR(rotation.Yaw(), 2.7, error);
}
