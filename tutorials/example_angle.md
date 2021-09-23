# Angle example

This tutorial explains how to use the angle class from ignition math library.

## CPP example

### Compile the code

To compile the code, go to `ign-math/examples` and use `cmake` to compile the code:

```{.sh}
cd ign-math/examples
mkdir build
cd build
cmake ..
make
```

When the code is compiled just run:

```{.sh}
./angle_example
```

The ouput of the program:

```{.sh}
The angle 'a' should be zero: 0
Pi in radians: 3.14159
Pi in degrees: 180
Pi + PI/2 in radians: 4.71239
Normalized to the range -Pi and Pi: -1.5708
```

### Code

The code declares an angle class. The default constructed angle should be zero.

```{.cpp}
// Create an angle.
ignition::math::Angle a;
```

There are some predefined angles such as:

```{.cpp}
// PI
ignition::math::Angle a = ignition::math::Angle::Pi;
ignition::math::Angle a_half = ignition::math::Angle::HalfPi;
```

By the default all the value are in radians, but you can use the method `Degree` to convert it to degrees.

```{.cpp}
// Output the angle in radians and degrees.
std::cout << "Pi in radians: " << a << std::endl;
std::cout << "Pi in degrees: " << a.Degree() << std::endl;
```

The Angle class overloads the +=, and many other, math operators.

```{.cpp}
a += ignition::math::Angle::HalfPi;
```

Use the method `Normalized` to bounded the value between `-PI` and `PI`

```{.cpp}
std::cout << "Normalized to the range -Pi and Pi: "
  << a.Normalized() << std::endl;
```

## Ruby example

This example will only work if the Ruby interface library was compiled and installed. Modify the `RUBYLIB` environment variable to include the ignition math library install path. For example, if you install to ``/usr`:

```{.sh}
export RUBYLIB=/usr/lib/ruby:$RUBYLIB
```

Execute the code:

```{.sh}
ruby angle_example.rb
```

### Code

There are some predefined values:

```{.rb}
printf("PI in degrees = %f\n", Ignition::Math::Angle.Pi.Degree)
```

Create new objects:


```{.rb}
a1 = Ignition::Math::Angle.new(1.5707)
a2 = Ignition::Math::Angle.new(0.7854)
```

Use the values in radians or degrees:

```{.rb}
printf("a1 = %f radians, %f degrees\n", a1.Radian, a1.Degree)
printf("a2 = %f radians, %f degrees\n", a2.Radian, a2.Degree)
```

The Angle class overloads math operators.

```{.rb}
printf("a1 * a2 = %f radians, %f degrees\n", (a1 * a2).Radian, (a1 * a2).Degree)
printf("a1 + a2 = %f radians, %f degrees\n", (a1 + a2).Radian, (a1 + a2).Degree)
printf("a1 - a2 = %f radians, %f degrees\n", (a1 - a2).Radian, (a1 - a2).Degree)
```

Normalize the value between `-PI` and `PI`.

```{.rb}
a3 = Ignition::Math::Angle.new(15.707)
printf("a3 = %f radians, %f degrees\n", a3.Radian, a3.Degree)
a3.Normalize
printf("a3.Normalize = %f radians, %f degrees\n", a3.Radian, a3.Degree)
```
