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
#ifndef GZ_MATH_SPHERICALCOORDINATES_HH_
#define GZ_MATH_SPHERICALCOORDINATES_HH_

#include <string>

#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {

    /// \brief Convert spherical coordinates for planetary surfaces.
    class GZ_MATH_VISIBLE SphericalCoordinates
    {
      /// \enum SurfaceType
      /// \brief Unique identifiers for planetary surface models.
      public: enum SurfaceType
              {
                /// \brief Model of reference ellipsoid for earth, based on
                /// WGS 84 standard. see wikipedia: World_Geodetic_System
                EARTH_WGS84 = 1,

                /// \brief Model of the moon, based on the Selenographic
                /// coordinate system, see wikipedia: Selenographic
                /// Coordinate System.
                MOON_SCS = 2,

                /// \brief Custom surface type
                CUSTOM_SURFACE = 10
              };

      /// \enum CoordinateType
      /// \brief Unique identifiers for coordinate types.
      public: enum CoordinateType
              {
                /// \brief Latitude, Longitude and Altitude by SurfaceType
                SPHERICAL = 1,

                /// \brief Earth centered, earth fixed Cartesian
                ECEF = 2,

                /// \brief Local tangent plane (East, North, Up)
                GLOBAL = 3,

                /// \brief Heading-adjusted tangent plane (X, Y, Z)
                /// This has kept a bug for backwards compatibility, use
                /// LOCAL2 for the correct behaviour.
                LOCAL = 4,

                /// \brief Heading-adjusted tangent plane (X, Y, Z)
                LOCAL2 = 5
              };

      /// \brief Constructor.
      public: SphericalCoordinates();

      /// \brief Constructor with surface type input.
      /// \param[in] _type SurfaceType specification.
      public: explicit SphericalCoordinates(const SurfaceType _type);

      /// \brief Constructor with surface type input and properties
      /// input. To be used for CUSTOM_SURFACE.
      /// \param[in] _type SurfaceType specification.
      /// \param[in] _axisEquatorial Semi major axis of the surface.
      /// \param[in] _axisPolar Semi minor axis of the surface.
      public: SphericalCoordinates(
            const SurfaceType _type,
            const double _axisEquatorial,
            const double _axisPolar);

      /// \brief Constructor with surface type, angle, and elevation inputs.
      /// \param[in] _type SurfaceType specification.
      /// \param[in] _latitude Reference latitude.
      /// \param[in] _longitude Reference longitude.
      /// \param[in] _elevation Reference elevation.
      /// \param[in] _heading Heading offset.
      public: SphericalCoordinates(const SurfaceType _type,
                                   const gz::math::Angle &_latitude,
                                   const gz::math::Angle &_longitude,
                                   const double _elevation,
                                   const gz::math::Angle &_heading);

      /// \brief Convert a Cartesian position vector to geodetic coordinates.
      /// This performs a `PositionTransform` from LOCAL to SPHERICAL.
      ///
      /// There's a known bug with this computation that can't be fixed on
      /// version 6 to avoid behaviour changes. Directly call
      /// `PositionTransform(_xyz, LOCAL2, SPHERICAL)` for correct results.
      /// Note that `PositionTransform` returns spherical coordinates in
      /// radians.
      ///
      /// \param[in] _xyz Cartesian position vector in the heading-adjusted
      /// world frame.
      /// \return Cooordinates: geodetic latitude (deg), longitude (deg),
      ///         altitude above sea level (m).
      public: gz::math::Vector3d SphericalFromLocalPosition(
                  const gz::math::Vector3d &_xyz) const;

      /// \brief Convert a Cartesian velocity vector in the local frame
      ///        to a global Cartesian frame with components East, North, Up.
      /// This is a wrapper around `VelocityTransform(_xyz, LOCAL, GLOBAL)`
      ///
      /// There's a known bug with this computation that can't be fixed on
      /// version 6 to avoid behaviour changes. Directly call
      /// `VelocityTransform(_xyz, LOCAL2, GLOBAL)` for correct results.
      ///
      /// \param[in] _xyz Cartesian velocity vector in the heading-adjusted
      /// world frame.
      /// \return Rotated vector with components (x,y,z): (East, North, Up).
      public: gz::math::Vector3d GlobalFromLocalVelocity(
                  const gz::math::Vector3d &_xyz) const;

      /// \brief Convert a string to a SurfaceType.
      /// Allowed values: ["EARTH_WGS84"].
      /// \param[in] _str String to convert.
      /// \return Conversion to SurfaceType.
      public: static SurfaceType Convert(const std::string &_str);

      /// \brief Convert a SurfaceType to a string.
      /// \param[in] _type Surface type
      /// \return Type as string
      public: static std::string Convert(SurfaceType _type);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _lonA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _lonB Longitude of point B.
      /// \return Distance in meters.
      /// \deprecated Use DistanceWGS84 instead.
      public: GZ_DEPRECATED(7) static double Distance(
                                     const gz::math::Angle &_latA,
                                     const gz::math::Angle &_lonA,
                                     const gz::math::Angle &_latB,
                                     const gz::math::Angle &_lonB);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// This method assumes that the surface model is EARTH_WGS84.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _lonA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _lonB Longitude of point B.
      /// \return Distance in meters.
      public: static double DistanceWGS84(
                                     const gz::math::Angle &_latA,
                                     const gz::math::Angle &_lonA,
                                     const gz::math::Angle &_latB,
                                     const gz::math::Angle &_lonB);

      /// \brief Get the distance between two points expressed in geographic
      /// latitude and longitude. It assumes that both points are at sea level.
      /// Example: _latA = 38.0016667 and _lonA = -123.0016667) represents
      /// the point with latitude 38d 0'6.00"N and longitude 123d 0'6.00"W.
      /// This is different from the deprecated static Distance() method as it
      /// takes into account the set surface's radius.
      /// \param[in] _latA Latitude of point A.
      /// \param[in] _lonA Longitude of point A.
      /// \param[in] _latB Latitude of point B.
      /// \param[in] _lonB Longitude of point B.
      /// \return Distance in meters.
      public: double DistanceBetweenPoints(
                  const gz::math::Angle &_latA,
                  const gz::math::Angle &_lonA,
                  const gz::math::Angle &_latB,
                  const gz::math::Angle &_lonB);

      /// \brief Get SurfaceType currently in use.
      /// \return Current SurfaceType value.
      public: SurfaceType Surface() const;

      /// \brief Get the radius of the surface.
      /// \return radius of the surface in use.
      public: double SurfaceRadius() const;

      /// \brief Get the major axis of the surface.
      /// \return Equatorial axis of the surface in use.
      public: double SurfaceAxisEquatorial() const;

      /// \brief Get the minor axis of the surface.
      /// \return Polar axis of the surface in use.
      public: double SurfaceAxisPolar() const;

      /// \brief Get the flattening of the surface.
      /// \return Flattening parameter of the surface in use.
      public: double SurfaceFlattening() const;

      /// \brief Get reference geodetic latitude.
      /// \return Reference geodetic latitude.
      public: gz::math::Angle LatitudeReference() const;

      /// \brief Get reference longitude.
      /// \return Reference longitude.
      public: gz::math::Angle LongitudeReference() const;

      /// \brief Get reference elevation in meters.
      /// \return Reference elevation.
      public: double ElevationReference() const;

      /// \brief Get heading offset for the reference frame, expressed as
      ///        angle from East to x-axis, or equivalently
      ///        from North to y-axis.
      /// \return Heading offset of reference frame.
      public: gz::math::Angle HeadingOffset() const;

      /// \brief Set SurfaceType for planetary surface model.
      /// \param[in] _type SurfaceType value.
      public: void SetSurface(const SurfaceType &_type);

      /// \brief Set SurfaceType for planetary surface model with
      /// custom ellipsoid properties.
      /// \param[in] _type SurfaceType value.
      /// \param[in] _axisEquatorial Equatorial axis of the surface.
      /// \param[in] _axisPolar Polar axis of the surface.
      public: void SetSurface(
                  const SurfaceType &_type,
                  const double _axisEquatorial,
                  const double _axisPolar);

      /// \brief Set reference geodetic latitude.
      /// \param[in] _angle Reference geodetic latitude.
      public: void SetLatitudeReference(const gz::math::Angle &_angle);

      /// \brief Set reference longitude.
      /// \param[in] _angle Reference longitude.
      public: void SetLongitudeReference(const gz::math::Angle &_angle);

      /// \brief Set reference elevation above sea level in meters.
      /// \param[in] _elevation Reference elevation.
      public: void SetElevationReference(const double _elevation);

      /// \brief Set heading angle offset for the frame.
      /// \param[in] _angle Heading offset for the frame.
      public: void SetHeadingOffset(const gz::math::Angle &_angle);

      /// \brief Convert a geodetic position vector to Cartesian coordinates.
      /// This performs a `PositionTransform` from SPHERICAL to LOCAL.
      /// \param[in] _latLonEle Geodetic position in the planetary frame of
      /// reference. X: latitude (deg), Y: longitude (deg), X: altitude.
      /// \return Cartesian position vector in the heading-adjusted world frame.
      public: gz::math::Vector3d LocalFromSphericalPosition(
                  const gz::math::Vector3d &_latLonEle) const;

      /// \brief Convert a Cartesian velocity vector with components East,
      /// North, Up to a local cartesian frame vector XYZ.
      /// This is a wrapper around `VelocityTransform(_xyz, GLOBAL, LOCAL)`
      /// \param[in] _xyz Vector with components (x,y,z): (East, North, Up).
      /// \return Cartesian vector in the world frame.
      public: gz::math::Vector3d LocalFromGlobalVelocity(
                  const gz::math::Vector3d &_xyz) const;

      /// \brief Update coordinate transformation matrix with reference location
      public: void UpdateTransformationMatrix();

      /// \brief Convert between positions in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// Spherical coordinates use radians, while the other frames use meters.
      /// \param[in] _pos Position vector in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed coordinate using cached origin.
      public: gz::math::Vector3d
              PositionTransform(const gz::math::Vector3d &_pos,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \brief Convert between velocity in SPHERICAL/ECEF/LOCAL/GLOBAL frame
      /// Spherical coordinates use radians, while the other frames use meters.
      /// \param[in] _vel Velocity vector in frame defined by parameter _in
      /// \param[in] _in  CoordinateType for input
      /// \param[in] _out CoordinateType for output
      /// \return Transformed velocity vector
      public: gz::math::Vector3d VelocityTransform(
                  const gz::math::Vector3d &_vel,
                  const CoordinateType &_in, const CoordinateType &_out) const;

      /// \brief Equality operator, result = this == _sc
      /// \param[in] _sc Spherical coordinates to check for equality
      /// \return true if this == _sc
      public: bool operator==(const SphericalCoordinates &_sc) const;

      /// \brief Inequality
      /// \param[in] _sc Spherical coordinates to check for inequality
      /// \return true if this != _sc
      public: bool operator!=(const SphericalCoordinates &_sc) const;

      /// \brief Pointer to the private data
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif
