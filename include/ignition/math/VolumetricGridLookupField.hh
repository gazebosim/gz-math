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

#ifndef IGNITION_MATH_VOLUMETRIC_GRID_LOOKUP_FIELD_HH_
#define IGNITION_MATH_VOLUMETRIC_GRID_LOOKUP_FIELD_HH_

#include <vector>
#include <optional>

#include <ignition/math/Vector3.hh>
#include <ignition/math/InterpolationPoint.hh>

#include <ignition/math/detail/AxisIndex.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
      template<typename T>
      /// \brief Lookup table for a volumetric dataset. This class is used to
      /// lookup indices for a large dataset that's organized in a grid. This
      /// class is not meant to be used with non-grid like data sets. The grid
      /// may be sparse or non uniform and missing data points.
      class VolumetricGridLookupField
      {
        /// Get the index along the given axis
        private: AxisIndex<T> z_indices_by_depth;

        private: AxisIndex<T> x_indices_by_lat;

        private: AxisIndex<T> y_indices_by_lon;

        private: std::vector<
          std::vector<std::vector<std::optional<std::size_t>>>> index_table;

        /// \brief Constructor
        /// \param[in] _cloud The cloud of points to use to construct the grid.
        public: VolumetricGridLookupField(
          const std::vector<Vector3<T>> &_cloud)
        {
          // NOTE: This part of the code assumes an exact grid of points.
          // The grid may be distorted or the stride between different points
          // may not be the same, but fundamentally the data is structured still
          // forms a grid-like structure. It keeps track of the axis indices for
          // each point in the grid.
          for(auto pt : _cloud)
          {
            x_indices_by_lat.AddIndexIfNotFound(pt.X());
            y_indices_by_lon.AddIndexIfNotFound(pt.Y());
            z_indices_by_depth.AddIndexIfNotFound(pt.Z());
          }

          int num_x = x_indices_by_lat.GetNumUniqueIndices();
          int num_y = y_indices_by_lon.GetNumUniqueIndices();
          int num_z = z_indices_by_depth.GetNumUniqueIndices();

          index_table.resize(num_z);
          for(int i = 0; i < num_z; ++i)
          {
            index_table[i].resize(num_y);
            for(int j = 0; j < num_y; ++j)
            {
              index_table[i][j].resize(num_x);
            }
          }

          for(std::size_t i = 0; i < _cloud.size(); ++i)
          {
            const auto &pt = _cloud[i];
            const std::size_t x_index =
              x_indices_by_lat.GetIndex(pt.X()).value();
            const std::size_t y_index =
              y_indices_by_lon.GetIndex(pt.Y()).value();
            const std::size_t z_index =
              z_indices_by_depth.GetIndex(pt.Z()).value();
            index_table[z_index][y_index][x_index] = i;
          }
        }

        /// \brief Retrieves the indices of the points that are to be used for
        /// interpolation.
        /// \param[in] _pt The point to get the interpolators for.
        /// \returns A list of points which are to be used for interpolation. If
        /// the point is in the middle of a grid cell then it returns the four
        /// corners of the grid cell. If its along a plane then it returns the
        /// four corners, if along an edge two points and if coincident with a
        /// corner one point. If the data is sparse and missing indices then a
        /// nullopt will be returned.
        public: std::vector<InterpolationPoint3D<T>>
          GetInterpolators(const Vector3<T> &_pt) const
        {
          std::vector<InterpolationPoint3D<T>> interpolators;

          auto x_indices = x_indices_by_lat.GetInterpolators(_pt.X());
          auto y_indices = y_indices_by_lon.GetInterpolators(_pt.Y());
          auto z_indices = z_indices_by_depth.GetInterpolators(_pt.Z());

          for(const auto &x_index : x_indices)
          {
            for(const auto &y_index : y_indices)
            {
              for(const auto &z_index : z_indices)
              {
                auto index = index_table[z_index.index][y_index.index][x_index.index];
                interpolators.push_back(
                  InterpolationPoint3D<T>{
                    Vector3<T>(
                      x_index.position,
                      y_index.position,
                      z_index.position
                    ),
                    std::optional{index}
                  });
              }
            }
          }

          return interpolators;
        }

        /// \brief Estimates the values for a grid given a list of values to
        /// interpolate. This method uses Trilinear interpolation.
        /// \param[in] _pt The point to estimate for.
        /// \param[in] _values The values to interpolate.
        /// \param[in] _default If a value is not found at a specific point then
        /// this value will be used.
        /// \returns The estimated value for the point.
        public: template<typename V> std::optional<V> EstimateValueUsingTrilinear(
          const Vector3<T> &_pt,
          const std::vector<V> &_values,
          const V &_default = V(0)) const
        {
          auto interpolators = GetInterpolators(_pt);
          if (interpolators.size() == 0)
          {
            return std::nullopt;
          }
          else if (interpolators.size() == 1)
          {
            if (!interpolators[0].index.has_value())
            {
              return _default;
            }
            return _values[interpolators[0].index.value()];
          }
          else if (interpolators.size() == 2)
          {
            return LinearInterpolate(interpolators[0], interpolators[1],
              _values, _pt, _default);
          }
          else if (interpolators.size() == 4)
          {
            return BiLinearInterpolate(interpolators, 0, _values, _pt, _default);
          }
          else if (interpolators.size() == 8)
          {
            return TrilinearInterpolate(interpolators, _values, _pt, _default);
          }
          else
          {
            return std::nullopt;
          }
        }

      };
    }
  }
}

#endif
