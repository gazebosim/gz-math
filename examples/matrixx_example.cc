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
#include <iostream>
#include <ignition/math/MatrixX.hh>

int main(int argc, char **argv)
{
  ignition::math::MatrixX<double, 2, 3> matd23A(
      0.1, 0.2, 0.3,
      0.4, 0.5, 0.6);

  ignition::math::MatrixX<double, 2, 3> matd23B(
      1.1, 1.2, 1.3,
      1.4, 1.5, 1.6);

  auto matSum = matd23A + matd23B;

  std::cout << matd23A << std::endl;

  std::cout << matd23B << std::endl;

  std::cout << matSum << std::endl;

  return 0;
}
