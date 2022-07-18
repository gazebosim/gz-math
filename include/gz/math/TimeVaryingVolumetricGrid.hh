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

#ifndef GZ_MATH_TIME_VARYING_VOLUMETRIC_GRID_HH_
#define GZ_MATH_TIME_VARYING_VOLUMETRIC_GRID_HH_

#include <gz/math/TimeVaryingVolumetricGridLookupField.hh>
#include <gz/math/Vector3.hh>
namespace gz
{
namespace math
{
template<typename T, typename V, typename S>
class TimeVaryingVolumetricGrid
{
  public: S CreateSession() const
  {
  }

  public: std::optional<S> StepTo(const S &_session, const T &_time) const
  {

  }

  public: std::optional<V> LookUp(const S &_session, const Vector3d &_pos) const
  {

  }
};

template<typename T, typename V>
class TimeVaryingVolumetricGrid<T, V, InMemorySession<T, double>>
{
  public: InMemorySession<T, V> CreateSession() const
  {
    return indices.CreateSession();
  }

  public: std::optional<InMemorySession<T, double>>
    StepTo(const InMemorySession<T, double> &_session, const T &_time) const
  {
    return indices.StepTo(_session, _time);
  }

  public: std::optional<V>
    LookUp(const InMemorySession<T, double> &_session, const Vector3d &_pos) 
    const
  {
    auto points = indices.LookUp(_session, _pos);
    V result = indices.EstimateQuadrilinear(
      _session,
      points,
      values,
      values,
      _pos);
    return result;
  }

  std::vector<V> values;
  TimeVaryingVolumetricGridLookupField<T, V, InMemorySession<T,double>> indices;
  template<typename U, typename S>
  friend class InMemoryTimeVaryingVolumetricGridFactory;
};

template<typename T, typename V>
class InMemoryTimeVaryingVolumetricGridFactory
{
  public: void AddPoint(
    const T &_time, const Vector3d &_position, const V &_value)
  {
    _points[_time].emplace_back(_position, _value);
  }

  public:
  TimeVaryingVolumetricGrid<T, V, InMemorySession<T,double>> Build() const
  {
    TimeVaryingVolumetricGrid<T, V, InMemorySession<T,double>> grid;
    for (auto &[time, pts]: _points)
    {
      std::vector<Vector3d> cloud;
      std::vector<std::size_t> indices;
      for (auto &[pt, val]: pts)
      {
        grid.values.push_back(val);
        cloud.push_back(pt);
        indices.push_back(grid.values.size() - 1);
      }
      VolumetricGridLookupField<V> field(cloud, indices);
      grid.indices.AddVolumetricGridField(time, field);
    }
    return grid;
  }

  private: std::map<T, std::vector<std::pair<Vector3d, V>>> _points;
};

}
}

#endif