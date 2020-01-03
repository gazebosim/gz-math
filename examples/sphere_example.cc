/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
//! [complete]
#include <stdio.h>
#include <ignition/math/Sphere.hh>

int main(int argc, char **argv)
{
  // Construct a sphere with a radius of 1.2 meters.
  ignition::math::Sphered sphere(1.2);

  printf("A sphere with radius %fm has a volume of %fm^3\n",
      sphere.Radius(), sphere.Volume());

  // Set the sphere's material properties
  sphere.SetMaterial(
      ignition::math::Material(ignition::math::MaterialType::PINE));

  // Get the mass matrix of the sphere when its material is pine.
  ignition::math::MassMatrix3d massMat;
  sphere.MassMatrix(massMat);
  printf("If this sphere is made of pine, then the mass is %fkg\n",
      massMat.Mass());
  printf("Pine has a density of %fkg/m^3\n", sphere.Material().Density());

  // Get the mass matrix of the sphere when its material is styrofoam.
  sphere.SetMaterial(
      ignition::math::Material(ignition::math::MaterialType::STYROFOAM));
  sphere.MassMatrix(massMat);
  printf("If this sphere is made of styrofoam, then the mass is %fkg\n",
      massMat.Mass());
  printf("Styrofoam has a density of %fkg/m^3\n", sphere.Material().Density());
}
//! [complete]
