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

#include "ignition/math/MovingWindowFilter.hh"
#include "ignition/math/Vector3.hh"

namespace ignition 
{
namespace math
{
inline namespace IGNITION_MATH_VERSION_NAMESPACE {

//////////////////////////////////////////////////
template<typename T>
MovingWindowFilter<T>::MovingWindowFilter(unsigned int _windowSize)
{
  this->SetWindowSize(_windowSize);
}

//////////////////////////////////////////////////
template<typename T>
void MovingWindowFilter<T>::Update(const T _val)
{
  // update sum and sample size with incoming _val

  // keep running sum
  this->sum += _val;

  // shift pointer, wrap around if end has been reached.
  ++this->valIter;
  if (this->valIter == this->valHistory.end())
  {
    // reset iterator to beginning of queue
    this->valIter = this->valHistory.begin();
  }

  // increment sample size
  ++this->samples;

  if (this->samples > this->valWindowSize)
  {
    // subtract old value if buffer already filled
    this->sum -= (*this->valIter);
    // put new value into queue
    (*this->valIter) = _val;
    // reduce sample size
    --this->samples;
  }
  else
  {
    // put new value into queue
    (*this->valIter) = _val;
  }
}

//////////////////////////////////////////////////
template<typename T>
void MovingWindowFilter<T>::SetWindowSize(const unsigned int _n)
{
  this->valWindowSize = _n;
  this->valHistory = std::vector<T>(_n, T());
  this->valIter = this->valHistory.begin();
  this->sum = T();
  this->samples = 0;
}

//////////////////////////////////////////////////
template<typename T>
unsigned int MovingWindowFilter<T>::WindowSize() const
{
  return this->valWindowSize;
}

//////////////////////////////////////////////////
template<typename T>
bool MovingWindowFilter<T>::WindowFilled() const
{
  return this->samples == this->valWindowSize;
}

//////////////////////////////////////////////////
template<typename T>
T MovingWindowFilter<T>::Value() const
{
  return this->sum / static_cast<double>(this->samples);
}

template class MovingWindowFilter<int>;
template class MovingWindowFilter<float>;
template class MovingWindowFilter<double>;
template class MovingWindowFilter<ignition::math::Vector3i>;
template class MovingWindowFilter<ignition::math::Vector3f>;
template class MovingWindowFilter<ignition::math::Vector3d>;

}
}
}
