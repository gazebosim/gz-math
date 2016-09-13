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
#ifndef IGNITION_MATH_VECTOR2_H_
#define IGNITION_MATH_VECTOR2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <ignition/math/System.hh>

typedef struct ign_vector2d ign_vector2d;

/**
 * \brief Default vector2 construction
 * \return Pointer to a ignition vector2d object.
 */
IGNITION_VISIBLE
ign_vector2d *ign_vector2d_create();

/**
 * \brief Create a vector2 initialized with two doubles
 * \param[in] _x X value
 * \param[in] _y Y value
 * \return Pointer to a ignition vector2d object.
 */
IGNITION_VISIBLE
ign_vector2d *ign_vector2d_create_double(double _x, double _y);

/**
 * \brief Create a copy of an existing vector2d object
 * \param[in] _pt Existing vector2d object.
 * \return Pointer to a new vector2d with values set to be identical
 * to _pt.
 */
IGNITION_VISIBLE
ign_vector2d *ign_vector2d_create_copy(const ign_vector2d *_pt);

/**
 * \brief Compute the distance between _pt1 and _pt2.
 * \param[in] _pt1 The first point.
 * \param[in] _pt2 The second point.
 * \return The distance between _pt1 and _pt2. IGN_DBL_MAX is returned if
 * _pt1 or _pt2 is null.
 */
IGNITION_VISIBLE
double ign_vector2d_distance(const ign_vector2d *_pt1,
                             const ign_vector2d *_pt2);

/**
 * \brief Returns the length (magnitude) of the vector
 * \param[in] _pt Pointer to the vector2d
 * \return The length of _pt. IGN_DBL_MAX is return if _pt is null.
 */
IGNITION_VISIBLE
double ign_vector2d_length(const ign_vector2d *_pt);

/**
 * \brief Return the x component value
 * \param[in] _pt Pointer to a vector2d
 * \return The x compoent of _pt. IGN_DBL_MAX is returned if _pt is null.
 */
IGNITION_VISIBLE
double ign_vector2d_x(const ign_vector2d *_pt);

/**
 * \brief Return the y component value
 * \param[in] _pt Pointer to a vector2d
 * \return The y compoent of _pt. IGN_DBL_MAX is returned if _pt is null.
 */
IGNITION_VISIBLE
double ign_vector2d_y(const ign_vector2d *_pt);

#ifdef __cplusplus
}
#endif

#endif
