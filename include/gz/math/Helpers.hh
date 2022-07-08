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
#ifndef GZ_MATH_FUNCTIONS_HH_
#define GZ_MATH_FUNCTIONS_HH_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ostream>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gz/math/config.hh>
#include "gz/math/Export.hh"

/// \brief The default tolerance value used by MassMatrix3::IsValid(),
/// MassMatrix3::IsPositive(), and MassMatrix3::ValidMoments()
template <typename T>
constexpr T GZ_MASSMATRIX3_DEFAULT_TOLERANCE = T(10);

// TODO(CH3): Deprecated. Remove on tock.
template <typename T>
constexpr T IGN_MASSMATRIX3_DEFAULT_TOLERANCE = T(10);

/// \brief Define GZ_PI, GZ_PI_2, and GZ_PI_4.
/// This was put here for Windows support.
#ifdef M_PI
#define GZ_PI M_PI
#define GZ_PI_2 M_PI_2
#define GZ_PI_4 M_PI_4
#define GZ_SQRT2 M_SQRT2
#else
#define GZ_PI   3.14159265358979323846
#define GZ_PI_2 1.57079632679489661923
#define GZ_PI_4 0.78539816339744830962
#define GZ_SQRT2 1.41421356237309504880
#endif

/// \brief Define GZ_FP_VOLATILE for FP equality comparisons
/// Use volatile parameters when checking floating point equality on
/// the 387 math coprocessor to work around bugs from the 387 extra precision
#if defined __FLT_EVAL_METHOD__  &&  __FLT_EVAL_METHOD__ == 2
#define GZ_FP_VOLATILE volatile
#else
#define GZ_FP_VOLATILE
#endif

/// \brief Compute sphere volume
/// \param[in] _radius Sphere radius
#define GZ_SPHERE_VOLUME(_radius) (4.0*GZ_PI*std::pow(_radius, 3)/3.0)

/// \brief Compute cylinder volume
/// \param[in] _r Cylinder base radius
/// \param[in] _l Cylinder length
#define GZ_CYLINDER_VOLUME(_r, _l) (_l * GZ_PI * std::pow(_r, 2))

/// \brief Compute box volume
/// \param[in] _x X length
/// \param[in] _y Y length
/// \param[in] _z Z length
#define GZ_BOX_VOLUME(_x, _y, _z) (_x *_y * _z)

/// \brief Compute box volume from a vector
/// \param[in] _v Vector3d that contains the box's dimensions.
#define GZ_BOX_VOLUME_V(_v) (_v.X() *_v.Y() * _v.Z())

namespace gz
{
  /// \brief Math classes and function useful in robot applications.
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \brief size_t type with a value of 0
    static const size_t GZ_ZERO_SIZE_T  = 0u;

    /// \brief size_t type with a value of 1
    static const size_t GZ_ONE_SIZE_T   = 1u;

    /// \brief size_t type with a value of 2
    static const size_t GZ_TWO_SIZE_T   = 2u;

    /// \brief size_t type with a value of 3
    static const size_t GZ_THREE_SIZE_T = 3u;

    /// \brief size_t type with a value of 4
    static const size_t GZ_FOUR_SIZE_T  = 4u;

    /// \brief size_t type with a value of 5
    static const size_t GZ_FIVE_SIZE_T  = 5u;

    /// \brief size_t type with a value of 6
    static const size_t GZ_SIX_SIZE_T   = 6u;

    /// \brief size_t type with a value of 7
    static const size_t GZ_SEVEN_SIZE_T = 7u;

    /// \brief size_t type with a value of 8
    static const size_t GZ_EIGHT_SIZE_T = 8u;

    /// \brief size_t type with a value of 9
    static const size_t GZ_NINE_SIZE_T  = 9u;

    // TODO(CH3): Deprecated. Remove on tock.
    constexpr auto GZ_DEPRECATED(7) IGN_ZERO_SIZE_T  = &GZ_ZERO_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_ONE_SIZE_T   = &GZ_ONE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_TWO_SIZE_T   = &GZ_TWO_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_THREE_SIZE_T = &GZ_THREE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_FOUR_SIZE_T  = &GZ_FOUR_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_FIVE_SIZE_T  = &GZ_FIVE_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_SIX_SIZE_T   = &GZ_SIX_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_SEVEN_SIZE_T = &GZ_SEVEN_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_EIGHT_SIZE_T = &GZ_EIGHT_SIZE_T;
    constexpr auto GZ_DEPRECATED(7) IGN_NINE_SIZE_T  = &GZ_NINE_SIZE_T;

    /// \brief Double maximum value. This value will be similar to 1.79769e+308
    static const double MAX_D = std::numeric_limits<double>::max();

    /// \brief Double min value. This value will be similar to 2.22507e-308
    static const double MIN_D = std::numeric_limits<double>::min();

    /// \brief Double low value, equivalent to -MAX_D
    static const double LOW_D = std::numeric_limits<double>::lowest();

    /// \brief Double positive infinite value
    static const double INF_D = std::numeric_limits<double>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const double NAN_D = std::numeric_limits<double>::quiet_NaN();

    /// \brief Float maximum value. This value will be similar to 3.40282e+38
    static const float MAX_F = std::numeric_limits<float>::max();

    /// \brief Float minimum value. This value will be similar to 1.17549e-38
    static const float MIN_F = std::numeric_limits<float>::min();

    /// \brief Float low value, equivalent to -MAX_F
    static const float LOW_F = std::numeric_limits<float>::lowest();

    /// \brief float positive infinite value
    static const float INF_F = std::numeric_limits<float>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const float NAN_F = std::numeric_limits<float>::quiet_NaN();

    /// \brief 16bit unsigned integer maximum value
    static const uint16_t MAX_UI16 = std::numeric_limits<uint16_t>::max();

    /// \brief 16bit unsigned integer minimum value
    static const uint16_t MIN_UI16 = std::numeric_limits<uint16_t>::min();

    /// \brief 16bit unsigned integer lowest value. This is equivalent to
    /// GZ_UINT16_MIN, and is defined here for completeness.
    static const uint16_t LOW_UI16 = std::numeric_limits<uint16_t>::lowest();

    /// \brief 16-bit unsigned integer positive infinite value
    static const uint16_t INF_UI16 = std::numeric_limits<uint16_t>::infinity();

    /// \brief 16bit unsigned integer maximum value
    static const int16_t MAX_I16 = std::numeric_limits<int16_t>::max();

    /// \brief 16bit unsigned integer minimum value
    static const int16_t MIN_I16 = std::numeric_limits<int16_t>::min();

    /// \brief 16bit unsigned integer lowest value. This is equivalent to
    /// GZ_INT16_MIN, and is defined here for completeness.
    static const int16_t LOW_I16 = std::numeric_limits<int16_t>::lowest();

    /// \brief 16-bit unsigned integer positive infinite value
    static const int16_t INF_I16 = std::numeric_limits<int16_t>::infinity();

    /// \brief 32bit unsigned integer maximum value
    static const uint32_t MAX_UI32 = std::numeric_limits<uint32_t>::max();

    /// \brief 32bit unsigned integer minimum value
    static const uint32_t MIN_UI32 = std::numeric_limits<uint32_t>::min();

    /// \brief 32bit unsigned integer lowest value. This is equivalent to
    /// GZ_UINT32_MIN, and is defined here for completeness.
    static const uint32_t LOW_UI32 = std::numeric_limits<uint32_t>::lowest();

    /// \brief 32-bit unsigned integer positive infinite value
    static const uint32_t INF_UI32 = std::numeric_limits<uint32_t>::infinity();

    /// \brief 32bit unsigned integer maximum value
    static const int32_t MAX_I32 = std::numeric_limits<int32_t>::max();

    /// \brief 32bit unsigned integer minimum value
    static const int32_t MIN_I32 = std::numeric_limits<int32_t>::min();

    /// \brief 32bit unsigned integer lowest value. This is equivalent to
    /// GZ_INT32_MIN, and is defined here for completeness.
    static const int32_t LOW_I32 = std::numeric_limits<int32_t>::lowest();

    /// \brief 32-bit unsigned integer positive infinite value
    static const int32_t INF_I32 = std::numeric_limits<int32_t>::infinity();

    /// \brief 64bit unsigned integer maximum value
    static const uint64_t MAX_UI64 = std::numeric_limits<uint64_t>::max();

    /// \brief 64bit unsigned integer minimum value
    static const uint64_t MIN_UI64 = std::numeric_limits<uint64_t>::min();

    /// \brief 64bit unsigned integer lowest value. This is equivalent to
    /// GZ_UINT64_MIN, and is defined here for completeness.
    static const uint64_t LOW_UI64 = std::numeric_limits<uint64_t>::lowest();

    /// \brief 64-bit unsigned integer positive infinite value
    static const uint64_t INF_UI64 = std::numeric_limits<uint64_t>::infinity();

    /// \brief 64bit unsigned integer maximum value
    static const int64_t MAX_I64 = std::numeric_limits<int64_t>::max();

    /// \brief 64bit unsigned integer minimum value
    static const int64_t MIN_I64 = std::numeric_limits<int64_t>::min();

    /// \brief 64bit unsigned integer lowest value. This is equivalent to
    /// GZ_INT64_MIN, and is defined here for completeness.
    static const int64_t LOW_I64 = std::numeric_limits<int64_t>::lowest();

    /// \brief 64-bit unsigned integer positive infinite value
    static const int64_t INF_I64 = std::numeric_limits<int64_t>::infinity();

    /// \brief Returns the representation of a quiet not a number (NAN)
    static const int NAN_I = std::numeric_limits<int>::quiet_NaN();

    /// \brief Simple clamping function that constrains a value to
    /// a range defined by a min and max value. This function is equivalent to
    /// std::max(std::min(value, max), min).
    /// \param[in] _v Value to clamp
    /// \param[in] _min Minimum allowed value.
    /// \param[in] _max Maximum allowed value.
    /// \return The value _v clamped to the range defined by _min and _max.
    template<typename T>
    inline T clamp(T _v, T _min, T _max)
    {
      return std::max(std::min(_v, _max), _min);
    }

    /// \brief Check if a float is NaN
    /// \param[in] _v The value to check.
    /// \return True if _v is not a number, false otherwise.
    inline bool isnan(float _v)
    {
      return (std::isnan)(_v);
    }

    /// \brief Check if a double is NaN.
    /// \param[in] _v The value to check
    /// \return True if _v is not a number, false otherwise.
    inline bool isnan(double _v)
    {
      return (std::isnan)(_v);
    }

    /// \brief Fix a float NaN value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN or infinite, _v otherwise.
    inline float fixnan(float _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0f : _v;
    }

    /// \brief Fix a double NaN value.
    /// \param[in] _v Value to correct.
    /// \return 0 if _v is NaN or is infinite, _v otherwise.
    inline double fixnan(double _v)
    {
      return isnan(_v) || std::isinf(_v) ? 0.0 : _v;
    }

    /// \brief Check if an int is even.
    /// \param[in] _v Value to check.
    /// \return True if _v is even.
    inline bool isEven(const int _v)
    {
      return !(_v % 2);
    }

    /// \brief Check if an unsigned int is even.
    /// \param[in] _v Value to check.
    /// \return True if _v is even.
    inline bool isEven(const unsigned int _v)
    {
      return !(_v % 2);
    }

    /// \brief Check if an int is odd.
    /// \param[in] _v Value to check.
    /// \return True if _v is odd.
    inline bool isOdd(const int _v)
    {
      return (_v % 2) != 0;
    }

    /// \brief Check if an unsigned int is odd.
    /// \param[in] _v Value to check.
    /// \return True if _v is odd.
    inline bool isOdd(const unsigned int _v)
    {
      return (_v % 2) != 0;
    }

    /// \brief The signum function.
    ///
    /// Returns 0 for zero values, -1 for negative values,
    /// +1 for positive values.
    /// \param[in] _value The value.
    /// \return The signum of the value.
    template<typename T>
    inline int sgn(T _value)
    {
      return (T(0) < _value) - (_value < T(0));
    }

    /// \brief The signum function.
    ///
    /// Returns 0 for zero values, -1 for negative values,
    /// +1 for positive values.
    /// \param[in] _value The value.
    /// \return The signum of the value.
    template<typename T>
    inline int signum(T _value)
    {
      return sgn(_value);
    }

    /// \brief Get mean value in a vector of values
    /// \param[in] _values The vector of values.
    /// \return The mean value in the provided vector.
    template<typename T>
    inline T mean(const std::vector<T> &_values)
    {
      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += _values[i];
      return sum / _values.size();
    }

    /// \brief Get the variance of a vector of values.
    /// \param[in] _values The vector of values.
    /// \return The squared deviation of the vector of values.
    template<typename T>
    inline T variance(const std::vector<T> &_values)
    {
      T avg = mean<T>(_values);

      T sum = 0;
      for (unsigned int i = 0; i < _values.size(); ++i)
        sum += (_values[i] - avg) * (_values[i] - avg);
      return sum / _values.size();
    }

    /// \brief Get the maximum value of vector of values.
    /// \param[in] _values The vector of values.
    /// \return Maximum value in the vector.
    template<typename T>
    inline T max(const std::vector<T> &_values)
    {
      return *std::max_element(std::begin(_values), std::end(_values));
    }

    /// \brief Get the minimum value of vector of values.
    /// \param[in] _values The vector of values.
    /// \return Minimum value in the vector.
    template<typename T>
    inline T min(const std::vector<T> &_values)
    {
      return *std::min_element(std::begin(_values), std::end(_values));
    }

    /// \brief Check if two values are equal, within a tolerance.
    /// \param[in] _a The first value.
    /// \param[in] _b The second value.
    /// \param[in] _epsilon The tolerance
    /// \return True if the two values fall within the given tolerance.
    template<typename T>
    inline bool equal(const T &_a, const T &_b,
                      const T &_epsilon = T(1e-6))
    {
      GZ_FP_VOLATILE T diff = std::abs(_a - _b);
      return diff <= _epsilon;
    }

    /// \brief Less than or near test, within a tolerance.
    /// \param[in] _a The first value.
    /// \param[in] _b The second value.
    /// \param[in] _epsilon The tolerance.
    /// \return True if _a < _b + _tol.
    template<typename T>
    inline bool lessOrNearEqual(const T &_a, const T &_b,
                            const T &_epsilon = 1e-6)
    {
      return _a < _b + _epsilon;
    }

    /// \brief Greater than or near test, within a tolerance.
    /// \param[in] _a The first value.
    /// \param[in] _b The second value.
    /// \param[in] _epsilon The tolerance.
    /// \return True if _a > _b - _epsilon.
    template<typename T>
    inline bool greaterOrNearEqual(const T &_a, const T &_b,
                               const T &_epsilon = 1e-6)
    {
      return _a > _b - _epsilon;
    }

    /// \brief Get the value at a specified precision.
    /// \param[in] _a The number.
    /// \param[in] _precision The precision.
    /// \return The value for the specified precision.
    template<typename T>
    inline T precision(const T &_a, const unsigned int &_precision)
    {
      auto p = std::pow(10, _precision);
      return static_cast<T>(std::round(_a * p) / p);
    }

    /// \brief Sort two numbers, such that _a <= _b.
    /// \param[in, out] _a The first number. This variable will contain the
    /// lower of the two values after this function completes.
    /// \param[in, out] _b The second number. This variable will contain the
    /// higher of the two values after this function completes.
    template<typename T>
    inline void sort2(T &_a, T &_b)
    {
      if (_b < _a)
        std::swap(_a, _b);
    }

    /// \brief Sort three numbers, such that _a <= _b <= _c.
    /// \param[in,out] _a The first number. This variable will contain the
    /// lowest of the three values after this function completes.
    /// \param[in,out] _b The second number. This variable will contain the
    /// middle of the three values after this function completes.
    /// \param[in,out] _c The third number. This variable will contain the
    /// highest of the three values after this function completes.
    template<typename T>
    inline void sort3(T &_a, T &_b, T &_c)
    {
      // _a <= _b
      sort2(_a, _b);
      // _a <= _c, _b <= _c
      sort2(_b, _c);
      // _a <= _b <= _c
      sort2(_a, _b);
    }

    /// \brief Append a number to a stream. Makes sure "-0" is returned as "0".
    /// \param[out] _out Output stream.
    /// \param[in] _number Number to append.
    template<typename T>
    inline void appendToStream(std::ostream &_out, T _number)
    {
      if (std::fpclassify(_number) == FP_ZERO)
      {
        _out << 0;
      }
      else
      {
        _out << _number;
      }
    }

    /// \brief Append a number to a stream, specialized for int.
    /// \param[out] _out Output stream.
    /// \param[in] _number Number to append.
    template<>
    inline void appendToStream(std::ostream &_out, int _number)
    {
      _out << _number;
    }

    /// \brief Is the parameter a power of 2?
    /// \param[in] _x The number to check.
    /// \return True if _x is a power of 2, false otherwise.
    inline bool isPowerOfTwo(unsigned int _x)
    {
      return ((_x != 0) && ((_x & (~_x + 1)) == _x));
    }

    /// \brief Get the smallest power of two that is greater than or equal to
    /// a given value.
    /// \param[in] _x The value which marks the lower bound of the result.
    /// \return The same value if _x is already a power of two. Otherwise,
    /// it returns the smallest power of two that is greater than _x
    inline unsigned int roundUpPowerOfTwo(unsigned int _x)
    {
      if (_x == 0)
        return 1;

      if (isPowerOfTwo(_x))
        return _x;

      while (_x & (_x - 1))
        _x = _x & (_x - 1);

      _x = _x << 1;

      return _x;
    }

    /// \brief Round a number up to the nearest multiple. For example, if
    /// the input number is 12 and the multiple is 10, the result is 20.
    /// If the input number is negative, then the nearest multiple will be
    /// greater than or equal to the input number. For example, if the input
    /// number is -9 and the multiple is 2 then the output is -8.
    /// \param[in] _num Input number to round up.
    /// \param[in] _multiple The multiple. If the multiple is <= zero, then
    /// the input number is returned.
    /// \return The nearest multiple of _multiple that is greater than
    /// or equal to _num.
    inline int roundUpMultiple(int _num, int _multiple)
    {
      if (_multiple == 0)
        return _num;

      int remainder = std::abs(_num) % _multiple;
      if (remainder == 0)
        return _num;

      if (_num < 0)
        return -(std::abs(_num) - remainder);
      else
        return _num + _multiple - remainder;
    }

    /// \brief Parse string into an integer.
    /// \param[in] _input The input string.
    /// \return An integer, or NAN_I if unable to parse the input.
    int GZ_MATH_VISIBLE parseInt(const std::string &_input);

    /// \brief parse string into float.
    /// \param [in] _input The string.
    /// \return A floating point number (can be NaN) or NAN_D if the
    /// _input could not be parsed.
    double GZ_MATH_VISIBLE parseFloat(const std::string &_input);

    /// \brief Convert a std::chrono::steady_clock::time_point to a seconds and
    /// nanoseconds pair.
    /// \param[in] _time The time point to convert.
    /// \return A pair where the first element is the number of seconds and
    /// the second is the number of nanoseconds.
    std::pair<int64_t, int64_t> GZ_MATH_VISIBLE timePointToSecNsec(
        const std::chrono::steady_clock::time_point &_time);

    /// \brief Convert seconds and nanoseconds to
    /// std::chrono::steady_clock::time_point.
    /// \param[in] _sec The seconds to convert.
    /// \param[in] _nanosec The nanoseconds to convert.
    /// \return A std::chrono::steady_clock::time_point based on the number of
    /// seconds and the number of nanoseconds.
    std::chrono::steady_clock::time_point GZ_MATH_VISIBLE
      secNsecToTimePoint(
        const uint64_t &_sec, const uint64_t &_nanosec);

    /// \brief Convert seconds and nanoseconds to
    /// std::chrono::steady_clock::duration.
    /// \param[in] _sec The seconds to convert.
    /// \param[in] _nanosec The nanoseconds to convert.
    /// \return A std::chrono::steady_clock::duration based on the number of
    /// seconds and the number of nanoseconds.
    std::chrono::steady_clock::duration GZ_MATH_VISIBLE secNsecToDuration(
        const uint64_t &_sec, const uint64_t &_nanosec);

    /// \brief Convert a std::chrono::steady_clock::duration to a seconds and
    /// nanoseconds pair.
    /// \param[in] _dur The duration to convert.
    /// \return A pair where the first element is the number of seconds and
    /// the second is the number of nanoseconds.
    std::pair<int64_t, int64_t> GZ_MATH_VISIBLE durationToSecNsec(
        const std::chrono::steady_clock::duration &_dur);

    // TODO(anyone): Replace this with std::chrono::days.
    /// This will exist in C++-20
    typedef std::chrono::duration<uint64_t, std::ratio<86400>> days;

    /// \brief break down durations
    /// NOTE: the template arguments must be properly ordered according
    /// to magnitude and there can be no duplicates.
    /// This function uses the braces initializer to split all the templated
    /// duration. The initializer will be called recursievely due the `...`
    /// \param[in] d Duration to break down
    /// \return A tuple based on the durations specified
    template<class...Durations, class DurationIn>
    std::tuple<Durations...> breakDownDurations(DurationIn d) {
      std::tuple<Durations...> retval;
      using discard = int[];
      (void)discard{0, (void((
        (std::get<Durations>(retval) =
          std::chrono::duration_cast<Durations>(d)),
        (d -= std::chrono::duration_cast<DurationIn>(
          std::get<Durations>(retval))))), 0)...};
      return retval;
    }

    /// \brief Convert a std::chrono::steady_clock::time_point to a string
    /// \param[in] _point The std::chrono::steady_clock::time_point to convert.
    /// \return A string formatted with the time_point
    std::string GZ_MATH_VISIBLE timePointToString(
        const std::chrono::steady_clock::time_point &_point);

    /// \brief Convert a std::chrono::steady_clock::duration to a string
    /// \param[in] _duration The std::chrono::steady_clock::duration to convert.
    /// \return A string formatted with the duration
    std::string GZ_MATH_VISIBLE durationToString(
        const std::chrono::steady_clock::duration &_duration);

    /// \brief Split a std::chrono::steady_clock::duration to a string
    /// \param[in] _timeString The string to convert in general format
    /// \param[out] numberDays number of days in the string
    /// \param[out] numberHours number of hours in the string
    /// \param[out] numberMinutes number of minutes in the string
    /// \param[out] numberSeconds number of seconds in the string
    /// \param[out] numberMilliseconds number of milliseconds in the string
    /// \return True if the regex was able to split the string otherwise False
    bool GZ_MATH_VISIBLE splitTimeBasedOnTimeRegex(
        const std::string &_timeString,
        uint64_t & numberDays, uint64_t & numberHours,
        uint64_t & numberMinutes, uint64_t & numberSeconds,
        uint64_t & numberMilliseconds);

    /// \brief Check if the given string represents a time.
    /// An example time string is "0 00:00:00.000", which has the format
    /// "DAYS HOURS:MINUTES:SECONDS.MILLISECONDS"
    /// \return True if the regex was able to split the string otherwise False
    inline bool isTimeString(const std::string &_timeString)
    {
      // These will be thrown away, just for making the function call
      uint64_t d, h, m, s, ms;
      return splitTimeBasedOnTimeRegex(_timeString, d, h, m, s, ms);
    }

    /// \brief Convert a string to a std::chrono::steady_clock::duration
    /// \param[in] _timeString The string to convert in general format
    /// "dd hh:mm:ss.nnn" where n is millisecond value
    /// \return A std::chrono::steady_clock::duration containing the
    /// string's time value. If it isn't possible to convert, the duration will
    /// be zero.
    std::chrono::steady_clock::duration GZ_MATH_VISIBLE stringToDuration(
        const std::string &_timeString);

    /// \brief Convert a string to a std::chrono::steady_clock::time_point
    /// \param[in] _timeString The string to convert in general format
    /// "dd hh:mm:ss.nnn" where n is millisecond value
    /// \return A std::chrono::steady_clock::time_point containing the
    /// string's time value. If it isn't possible to convert, the time will
    /// be negative 1 second.
    std::chrono::steady_clock::time_point
    GZ_MATH_VISIBLE stringToTimePoint(const std::string &_timeString);

    // Degrade precision on Windows, which cannot handle 'long double'
    // values properly. See the implementation of Unpair.
    // 32 bit ARM processors also define 'long double' to be the same
    // size as 'double', and must also be degraded
#if defined _MSC_VER || defined __arm__
    using PairInput = uint16_t;
    using PairOutput = uint32_t;
#else
    using PairInput = uint32_t;
    using PairOutput = uint64_t;
#endif

    /// \brief A pairing function that maps two values to a unique third
    /// value. This is an implementation of Szudzik's function.
    /// \param[in] _a First value, must be a non-negative integer. On
    /// Windows this value is uint16_t. On Linux/OSX this value is uint32_t.
    /// \param[in] _b Second value, must be a non-negative integer. On
    /// Windows this value is uint16_t. On Linux/OSX this value is uint32_t.
    /// \return A unique non-negative integer value. On Windows the return
    /// value is uint32_t. On Linux/OSX the return value is uint64_t
    /// \sa Unpair
    PairOutput GZ_MATH_VISIBLE Pair(
        const PairInput _a, const PairInput _b);

    /// \brief The reverse of the Pair function. Accepts a key, produced
    /// from the Pair function, and returns a tuple consisting of the two
    /// non-negative integer values used to create the _key.
    /// \param[in] _key A non-negative integer generated from the Pair
    /// function. On Windows this value is uint32_t. On Linux/OSX, this
    /// value is uint64_t.
    /// \return A tuple that consists of the two non-negative integers that
    /// will generate _key when used with the Pair function. On Windows the
    /// tuple contains two uint16_t values. On Linux/OSX the tuple contains
    /// two uint32_t values.
    /// \sa Pair
    std::tuple<PairInput, PairInput> GZ_MATH_VISIBLE Unpair(
        const PairOutput _key);
    }
  }
}

#endif
