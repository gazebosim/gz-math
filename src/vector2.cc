/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "ignition/math/vector2.h"
#include "ignition/math/Helpers.hh"
#include "ignition/math/Vector2.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
ign_vector2d *ign_vector2d_create()
{
  return reinterpret_cast<ign_vector2d*>(new Vector2d);
}

/////////////////////////////////////////////////
ign_vector2d *ign_vector2d_create_double(double _x, double _y)
{
  return reinterpret_cast<ign_vector2d*>(new Vector2d(_x, _y));
}

/////////////////////////////////////////////////
ign_vector2d *vector2d_create_copy(const ign_vector2d *_pt)
{
  if (_pt)
  {
    return reinterpret_cast<ign_vector2d*>(new Vector2d(
          *reinterpret_cast<const Vector2d*>(_pt)));
  }

  return NULL;
}

/////////////////////////////////////////////////
double ign_vector2d_distance(const ign_vector2d *_pt1, const ign_vector2d *_pt2)
{
  if (_pt1 && _pt2)
  {
    return reinterpret_cast<const Vector2d*>(_pt1)->Distance(
        *reinterpret_cast<const Vector2d*>(_pt2));
  }

  return IGN_DBL_MAX;
}

/////////////////////////////////////////////////
double ign_vector2d_length(const ign_vector2d *_pt)
{
  if (_pt)
    return reinterpret_cast<const Vector2d*>(_pt)->Length();

  return IGN_DBL_MAX;
}

/////////////////////////////////////////////////
double ign_vector2d_x(const ign_vector2d *_pt)
{
  if (_pt)
    return reinterpret_cast<const Vector2d*>(_pt)->X();
  return IGN_DBL_MAX;
}

/////////////////////////////////////////////////
double ign_vector2d_y(const ign_vector2d *_pt)
{
  return reinterpret_cast<const Vector2d*>(_pt)->Y();
}
