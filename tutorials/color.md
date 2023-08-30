\page color Color example

This tutorial explains how to use the `Color` class from Gazebo Math library.

## Compile the code

Go to `gz-math/examples` and use `cmake` to compile the code:

```{.sh}
git clone https://github.com/gazebosim/gz-math/ -b gz-math7
cd gz-math/examples
mkdir build
cd build
cmake ..
make
```

When the code is compiled, run:

```{.sh}
./color_example
```

The ouput of the program:

```{.sh}
The alpha value of a should be 1: 1
The RGBA value of a: 0.6 0.7 0.8 1
Check if a is Blue: 1
The RGB value of a should be (0, 0, 1): 0, 0, 1
The HSV value of a: 240 1 1
The RGBA value of a should be (0, 0, 1, 1): 0 0 1 1
```

## Code

Create a color with the following RGBA value. The the alpha value is set default to 1.0:

\snippet examples/color_example.cc Create a color

Change the value of a color via `Set()`. All values are set default to 1.0 if not specify.

\snippet examples/color_example.cc Set a new color value

The ABGR, ARGB, BGRA, RGBA are 4 datatypes that allow you to set color from a 32-bit int. Take BGRA as an example:

\snippet examples/color_example.cc Set value from BGRA

Color class overloads math operators including `+`, `-`, `*`, `/`, `[]`, and `==`.

\snippet examples/color_example.cc Math operator

You can also set or read a color in HSV.

\snippet examples/color_example.cc Set value from HSV

There are more functions in `Color`. Take a look at the [API](https://gazebosim.org/api/math/6.9/classignition_1_1math_1_1Color.html)
