/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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

#include "gz/math/Temperature.hh"

#include <istream>
#include <ostream>

/// \brief Private data for the Temperature class.
class gz::math::Temperature::Implementation
{
  /// \brief Temperature value in Kelvin.
  public: double kelvin{0.0};
};

using namespace gz;
using namespace math;

/////////////////////////////////////////////////
Temperature::Temperature()
: dataPtr(gz::utils::MakeImpl<Implementation>())
{
}

/////////////////////////////////////////////////
Temperature::Temperature(double _temp)
: Temperature()
{
  this->dataPtr->kelvin = _temp;
}

/////////////////////////////////////////////////
double Temperature::KelvinToCelsius(double _temp)
{
  return _temp - 273.15;
}

/////////////////////////////////////////////////
double Temperature::KelvinToFahrenheit(double _temp)
{
  return _temp * 1.8 - 459.67;
}

/////////////////////////////////////////////////
double Temperature::CelsiusToFahrenheit(double _temp)
{
  return _temp * 1.8 + 32.0;
}

/////////////////////////////////////////////////
double Temperature::CelsiusToKelvin(double _temp)
{
  return _temp + 273.15;
}

/////////////////////////////////////////////////
double Temperature::FahrenheitToCelsius(double _temp)
{
  return (_temp - 32.0) / 1.8;
}

/////////////////////////////////////////////////
double Temperature::FahrenheitToKelvin(double _temp)
{
  return (_temp + 459.67) / 1.8;
}

/////////////////////////////////////////////////
void Temperature::SetKelvin(double _temp)
{
  this->dataPtr->kelvin = _temp;
}

/////////////////////////////////////////////////
void Temperature::SetCelsius(double _temp)
{
  this->SetKelvin(CelsiusToKelvin(_temp));
}

/////////////////////////////////////////////////
void Temperature::SetFahrenheit(double _temp)
{
  this->SetKelvin(FahrenheitToKelvin(_temp));
}

/////////////////////////////////////////////////
double Temperature::Kelvin() const
{
  return this->dataPtr->kelvin;
}

/////////////////////////////////////////////////
double Temperature::Celsius() const
{
  return KelvinToCelsius(this->dataPtr->kelvin);
}

/////////////////////////////////////////////////
double Temperature::Fahrenheit() const
{
  return KelvinToFahrenheit(this->dataPtr->kelvin);
}

/////////////////////////////////////////////////
double Temperature::operator()() const
{
  return this->dataPtr->kelvin;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator=(double _temp)
{
  this->SetKelvin(_temp);
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator+(double _temp) const
{
  return this->dataPtr->kelvin + _temp;
}

/////////////////////////////////////////////////
Temperature Temperature::operator+(const Temperature &_temp) const
{
  return this->dataPtr->kelvin + _temp;
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator+=(double _temp)
{
  this->dataPtr->kelvin += _temp;
  return *this;
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator+=(const Temperature &_temp)
{
  this->dataPtr->kelvin += _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator-(double _temp) const
{
  return this->dataPtr->kelvin - _temp;
}

/////////////////////////////////////////////////
Temperature Temperature::operator-(const Temperature &_temp) const
{
  return this->dataPtr->kelvin - _temp.Kelvin();
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator-=(double _temp)
{
  this->dataPtr->kelvin -= _temp;
  return *this;
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator-=(const Temperature &_temp)
{
  this->dataPtr->kelvin -= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator*(double _temp) const
{
  return Temperature(this->dataPtr->kelvin * _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator*(const Temperature &_temp) const
{
  return Temperature(this->dataPtr->kelvin * _temp.Kelvin());
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator*=(double _temp)
{
  this->dataPtr->kelvin *= _temp;
  return *this;
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator*=(const Temperature &_temp)
{
  this->dataPtr->kelvin *= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator/(double _temp) const
{
  return Temperature(this->dataPtr->kelvin / _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator/(const Temperature &_temp) const
{
  return Temperature(this->dataPtr->kelvin / _temp.Kelvin());
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator/=(double _temp)
{
  this->dataPtr->kelvin /= _temp;
  return *this;
}

/////////////////////////////////////////////////
const Temperature &Temperature::operator/=(const Temperature &_temp)
{
  this->dataPtr->kelvin /= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
bool Temperature::operator==(const Temperature &_temp) const
{
  return equal(this->dataPtr->kelvin, _temp.Kelvin());
}

/////////////////////////////////////////////////
bool Temperature::operator==(double _temp) const
{
  return equal(this->dataPtr->kelvin, _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator!=(const Temperature &_temp) const
{
  return !(*this == _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator!=(double _temp) const
{
  return !(*this == _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator<(const Temperature &_temp) const
{
  return this->dataPtr->kelvin < _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator<(double _temp) const
{
  return this->dataPtr->kelvin < _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator<=(const Temperature &_temp) const
{
  return this->dataPtr->kelvin <= _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator<=(double _temp) const
{
  return this->dataPtr->kelvin <= _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator>(const Temperature &_temp) const
{
  return this->dataPtr->kelvin > _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator>(double _temp) const
{
  return this->dataPtr->kelvin > _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator>=(const Temperature &_temp) const
{
  return this->dataPtr->kelvin >= _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator>=(double _temp) const
{
  return this->dataPtr->kelvin >= _temp;
}
