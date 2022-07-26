/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gz/math/eigen3/Conversions.hh>

/////////////////////////////////////////////////
/// Check Vector3 conversions
TEST(EigenConversions, ConvertVector3)
{
  {
    gz::math::Vector3d iVec, iVec2;
    Eigen::Vector3d eVec = gz::math::eigen3::convert(iVec);
    EXPECT_DOUBLE_EQ(0, eVec[0]);
    EXPECT_DOUBLE_EQ(0, eVec[1]);
    EXPECT_DOUBLE_EQ(0, eVec[2]);
    iVec2 = gz::math::eigen3::convert(eVec);
    EXPECT_EQ(iVec, iVec2);
  }

  {
    gz::math::Vector3d iVec(100.5, -2.314, 42), iVec2;
    Eigen::Vector3d eVec = gz::math::eigen3::convert(iVec);
    EXPECT_DOUBLE_EQ(iVec[0], eVec[0]);
    EXPECT_DOUBLE_EQ(iVec[1], eVec[1]);
    EXPECT_DOUBLE_EQ(iVec[2], eVec[2]);
    iVec2 = gz::math::eigen3::convert(eVec);
    EXPECT_EQ(iVec, iVec2);
  }
}

/////////////////////////////////////////////////
/// Check AxisAlignedBox conversions
TEST(EigenConversions, ConvertAxisAlignedBox)
{
  {
    gz::math::AxisAlignedBox iBox, iBox2;
    Eigen::AlignedBox3d eBox = gz::math::eigen3::convert(iBox);
    EXPECT_DOUBLE_EQ(gz::math::MAX_D, eBox.min()[0]);
    EXPECT_DOUBLE_EQ(gz::math::MAX_D, eBox.min()[1]);
    EXPECT_DOUBLE_EQ(gz::math::MAX_D, eBox.min()[2]);
    EXPECT_DOUBLE_EQ(gz::math::LOW_D, eBox.max()[0]);
    EXPECT_DOUBLE_EQ(gz::math::LOW_D, eBox.max()[1]);
    EXPECT_DOUBLE_EQ(gz::math::LOW_D, eBox.max()[2]);
    iBox2 = gz::math::eigen3::convert(eBox);
    EXPECT_EQ(iBox, iBox2);
  }

  {
    gz::math::AxisAlignedBox iBox(
        gz::math::Vector3d(100.5, -2.314, 42),
        gz::math::Vector3d(305, 2.314, 142));
    gz::math::AxisAlignedBox iBox2;
    Eigen::AlignedBox3d eBox = gz::math::eigen3::convert(iBox);
    EXPECT_DOUBLE_EQ(iBox.Min()[0], eBox.min()[0]);
    EXPECT_DOUBLE_EQ(iBox.Min()[1], eBox.min()[1]);
    EXPECT_DOUBLE_EQ(iBox.Min()[2], eBox.min()[2]);
    EXPECT_DOUBLE_EQ(iBox.Max()[0], eBox.max()[0]);
    EXPECT_DOUBLE_EQ(iBox.Max()[1], eBox.max()[1]);
    EXPECT_DOUBLE_EQ(iBox.Max()[2], eBox.max()[2]);
    iBox2 = gz::math::eigen3::convert(eBox);
    EXPECT_EQ(iBox, iBox2);
  }
}

/////////////////////////////////////////////////
/// Check Quaternion conversions
TEST(EigenConversions, ConvertQuaternion)
{
  {
    gz::math::Quaterniond iQuat, iQuat2;
    Eigen::Quaterniond eQuat = gz::math::eigen3::convert(iQuat);
    EXPECT_DOUBLE_EQ(1, eQuat.w());
    EXPECT_DOUBLE_EQ(0, eQuat.x());
    EXPECT_DOUBLE_EQ(0, eQuat.y());
    EXPECT_DOUBLE_EQ(0, eQuat.z());
    iQuat2 = gz::math::eigen3::convert(eQuat);
    EXPECT_EQ(iQuat, iQuat2);
  }

  {
    gz::math::Quaterniond iQuat(0.1, 0.2, 0.3), iQuat2;
    Eigen::Quaterniond eQuat = gz::math::eigen3::convert(iQuat);
    EXPECT_DOUBLE_EQ(iQuat.W(), eQuat.w());
    EXPECT_DOUBLE_EQ(iQuat.X(), eQuat.x());
    EXPECT_DOUBLE_EQ(iQuat.Y(), eQuat.y());
    EXPECT_DOUBLE_EQ(iQuat.Z(), eQuat.z());
    iQuat2 = gz::math::eigen3::convert(eQuat);
    EXPECT_EQ(iQuat, iQuat2);
  }
}

/////////////////////////////////////////////////
/// Check Matrix3 conversions
TEST(EigenConversions, ConvertMatrix3)
{
  {
    gz::math::Matrix3d iMat, iMat2;
    Eigen::Matrix3d eMat = gz::math::eigen3::convert(iMat);
    EXPECT_DOUBLE_EQ(0, eMat(0, 0));
    EXPECT_DOUBLE_EQ(0, eMat(0, 1));
    EXPECT_DOUBLE_EQ(0, eMat(0, 2));
    EXPECT_DOUBLE_EQ(0, eMat(1, 0));
    EXPECT_DOUBLE_EQ(0, eMat(1, 1));
    EXPECT_DOUBLE_EQ(0, eMat(1, 2));
    EXPECT_DOUBLE_EQ(0, eMat(2, 0));
    EXPECT_DOUBLE_EQ(0, eMat(2, 1));
    EXPECT_DOUBLE_EQ(0, eMat(2, 2));
    iMat2 = gz::math::eigen3::convert(eMat);
    EXPECT_EQ(iMat, iMat2);
  }


  {
    gz::math::Matrix3d iMat(1, 2, 3, 4, 5, 6, 7, 8, 9), iMat2;
    Eigen::Matrix3d eMat = gz::math::eigen3::convert(iMat);
    EXPECT_DOUBLE_EQ(iMat(0, 0), eMat(0, 0));
    EXPECT_DOUBLE_EQ(iMat(0, 1), eMat(0, 1));
    EXPECT_DOUBLE_EQ(iMat(0, 2), eMat(0, 2));
    EXPECT_DOUBLE_EQ(iMat(1, 0), eMat(1, 0));
    EXPECT_DOUBLE_EQ(iMat(1, 1), eMat(1, 1));
    EXPECT_DOUBLE_EQ(iMat(1, 2), eMat(1, 2));
    EXPECT_DOUBLE_EQ(iMat(2, 0), eMat(2, 0));
    EXPECT_DOUBLE_EQ(iMat(2, 1), eMat(2, 1));
    EXPECT_DOUBLE_EQ(iMat(2, 2), eMat(2, 2));
    iMat2 = gz::math::eigen3::convert(eMat);
    EXPECT_EQ(iMat, iMat2);
  }
}

/////////////////////////////////////////////////
/// Check Matrix6 conversions
TEST(EigenConversions, ConvertMatrix6)
{
  {
    gz::math::Matrix6d iMat, iMat2;
    auto eMat = gz::math::eigen3::convert(iMat);
    for (std::size_t i = 0; i < 6; ++i)
    {
      for (std::size_t j = 0; j < 6; ++j)
      {
        EXPECT_DOUBLE_EQ(0.0, eMat(i, j));
      }
    }
    iMat2 = gz::math::eigen3::convert(eMat);
    EXPECT_EQ(iMat, iMat2);
  }

  {
    gz::math::Matrix6d iMat(
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0,
        6.0, 7.0, 8.0, 9.0, 10.0, 11.0,
        12.0, 13.0, 14.0, 15.0, 16.0, 17.0,
        18.0, 19.0, 20.0, 21.0, 22.0, 23.0,
        24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
        30.0, 31.0, 32.0, 33.0, 34.0, 35.0);
    gz::math::Matrix6d iMat2;
    auto eMat = gz::math::eigen3::convert(iMat);
    for (std::size_t i = 0; i < 6; ++i)
    {
      for (std::size_t j = 0; j < 6; ++j)
      {
        EXPECT_DOUBLE_EQ(iMat(i, j), eMat(i, j));
      }
    }
    iMat2 = gz::math::eigen3::convert(eMat);
    EXPECT_EQ(iMat, iMat2);
  }
}

/////////////////////////////////////////////////
/// Check Pose conversions
TEST(EigenConversions, ConvertPose3)
{
  {
    gz::math::Pose3d iPose, iPose2;
    Eigen::Isometry3d ePose = gz::math::eigen3::convert(iPose);
    Eigen::Vector3d eVec = ePose.translation();
    EXPECT_DOUBLE_EQ(0, eVec[0]);
    EXPECT_DOUBLE_EQ(0, eVec[1]);
    EXPECT_DOUBLE_EQ(0, eVec[2]);
    Eigen::Quaterniond eQuat(ePose.linear());
    EXPECT_DOUBLE_EQ(1, eQuat.w());
    EXPECT_DOUBLE_EQ(0, eQuat.x());
    EXPECT_DOUBLE_EQ(0, eQuat.y());
    EXPECT_DOUBLE_EQ(0, eQuat.z());
    iPose2 = gz::math::eigen3::convert(ePose);
    EXPECT_EQ(iPose, iPose2);
  }

  {
    gz::math::Pose3d iPose(105.4, 3.1, -0.34, 3.14/8, 3.14/16, -3.14/2);
    gz::math::Pose3d iPose2;
    Eigen::Isometry3d ePose = gz::math::eigen3::convert(iPose);
    Eigen::Vector3d eVec = ePose.translation();
    EXPECT_DOUBLE_EQ(iPose.Pos()[0], eVec[0]);
    EXPECT_DOUBLE_EQ(iPose.Pos()[1], eVec[1]);
    EXPECT_DOUBLE_EQ(iPose.Pos()[2], eVec[2]);
    Eigen::Quaterniond eQuat(ePose.linear());
    EXPECT_DOUBLE_EQ(iPose.Rot().W(), eQuat.w());
    EXPECT_DOUBLE_EQ(iPose.Rot().X(), eQuat.x());
    EXPECT_DOUBLE_EQ(iPose.Rot().Y(), eQuat.y());
    EXPECT_DOUBLE_EQ(iPose.Rot().Z(), eQuat.z());
    iPose2 = gz::math::eigen3::convert(ePose);
    EXPECT_EQ(iPose, iPose2);
  }
}
