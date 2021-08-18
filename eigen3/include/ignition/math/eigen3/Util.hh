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

#ifndef IGNITION_MATH_EIGEN3_UTIL_HH_
#define IGNITION_MATH_EIGEN3_UTIL_HH_

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/eigen3/Conversions.hh>
#include <ignition/math/OrientedBox.hh>

namespace ignition
{
  namespace math
  {
    namespace eigen3
    {
      inline Eigen::Matrix3d computeCovarianceMatrix(
        const std::vector<math::Vector3d> &mesh,
        const Eigen::Vector3d &centroid)
      {
        Eigen::Matrix3d covariance;
        covariance.setZero ();

        // For each 3d vertex in the mesh
        for (auto point : mesh)
        {
          auto pt = convert(point);
          pt = pt - centroid;

          covariance(1, 1) += pt.y() * pt.y();
          covariance(1, 2) += pt.y() * pt.z();
          covariance(2, 2) += pt.z() * pt.z();

          pt *= pt.x ();
          covariance(0, 0) += pt.x();
          covariance(0, 1) += pt.y();
          covariance(0, 2) += pt.z();
        }

        covariance(1, 0) = covariance(0, 1);
        covariance(2, 0) = covariance(0, 2);
        covariance(2, 1) = covariance(1, 2);

        return covariance;
      }

      /// \brief Get the oriented 3d bounding box of a mesh points using PCA
      inline ignition::math::OrientedBoxd MeshToOrientedBox(
        const std::vector<math::Vector3d> &_mesh)
      {
        math::OrientedBoxd box;

        // Centroid (mean) of the mesh
        math::Vector3d mean;
        for (auto point : _mesh)
          mean += point;
        mean /= _mesh.size();

        Eigen::Vector3d centroid = convert(mean);

        // Covariance
        Eigen::Matrix3d covariance = computeCovarianceMatrix(_mesh, centroid);

        // Eigen Vectors
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
          eigenSolver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3d eigenVectorsPCA = eigenSolver.eigenvectors();

        // This line is necessary for proper orientation in some cases.
        // The numbers come out the same without it, but The signs are
        // different and the box doesn't get correctly oriented in some cases.
        eigenVectorsPCA.col(2) =
          eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        // Transform the original cloud to the origin where the principal
        // components correspond to the axes.
        Eigen::Matrix4d projectionTransform(Eigen::Matrix4d::Identity());
        projectionTransform.block<3, 3> (0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1> (0, 3) =
          -1.f * (projectionTransform.block<3, 3>(0, 0) * centroid);

        Eigen::Vector3d minPoint(INF_I32, INF_I32, INF_I32);
        Eigen::Vector3d maxPoint(-INF_I32, -INF_I32, -INF_I32);

        // Get the minimum and maximum points of the transformed cloud.
        for (auto point : _mesh)
        {
          Eigen::Vector4d pt(0, 0, 0, 1);
          pt.head<3>() = convert(point);
          Eigen::Vector4d tfPoint = projectionTransform * pt;
          minPoint = minPoint.cwiseMin(tfPoint.head<3>());
          maxPoint = maxPoint.cwiseMax(tfPoint.head<3>());
        }

        const Eigen::Vector3d meanDiagonal = 0.5f*(maxPoint + minPoint);

        // quaternion is calculated using the eigenvectors (which determines
        // how the final box gets rotated), and the transform to put the box
        // in correct location is calculated
        const Eigen::Quaterniond bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3d bboxTransform =
          eigenVectorsPCA * meanDiagonal + centroid;

        // xyz xzy ... yxz yzx ... zyx zxy
        math::Vector3d size(
            maxPoint.x() - minPoint.x(),
            maxPoint.y() - minPoint.y(),
            maxPoint.z() - minPoint.z()
        );
        math::Pose3d pose;
        pose.Rot() = convert(bboxQuaternion);
        pose.Pos() = convert(bboxTransform);

        box.Size(size);
        box.Pose(pose);
        return box;
      }
    }
  }
}

#endif
