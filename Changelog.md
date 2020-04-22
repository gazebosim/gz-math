## Ignition Math 4.x

### Ignition Math 4.x.x

1. Add Graph::EdgeFromVertices function that return an edge, if one exists,
   between two vertices.
    * [BitBucket pull request 254](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/254)

1. Added multiply assign operator to Matrix4. 
    * [BitBucket pull request 252](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/252)


1. Add Plane copy constructor and fix cppcheck on artful
    * [BitBucket pull request 230](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/230)

1. Added MovingWindowFilter, a copy from Ignition Common. This version will
   replace the version found in Ignition Common.
    * [BitBucket pull request 239](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/239)

1. Added a Material class, which holds information about materials like wood,
   steel, and iron.
    * [BitBucket pull request 243](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/243)

### Ignition Math 4.0.0 (2017-12-26)

1. Use std::stoi and std::stod in math::parse* functions to reduce code
    * [BitBucket pull request 224](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/224)
    * [Issue 50](https://github.com/ignitionrobotics/ign-math/issues/50)

1. Fixing const-correctness for operator* of Pose3
    * [BitBucket pull request 205](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/205)

1. Deprecate Matrix4::Translate and replace by Matrix4::SetTranslation
    * [BitBucket pull request 222](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/222)

1. Use ignition-cmake to simplify build scripts
    * [BitBucket pull request 200](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/200)

1. Make constructor SemanticVersion(string) explicit
    * [BitBucket pull request 203](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/203)

1. Switch to C++14
    * [BitBucket pull request 180](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/180)

1. Removed the box 'extent' field. The default constructor now sets a box's
   corners to extrema in order to indicate an uninitialized box.
    * [BitBucket pull request 172](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/172)
    * [Issue 72](https://github.com/ignitionrobotics/ign-math/issues/72)
    * [Issue 53](https://github.com/ignitionrobotics/ign-math/issues/53)

1. Added graph utilites:
    1. Added a Vertex class:
    * [BitBucket pull request 170](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/170)
    1. Added an Edge class:
    * [BitBucket pull request 174](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/174)
    1. Added a Graph class:
    * [BitBucket pull request 175](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/175)
    1. Added a GraphAlgorithms class:
    * [BitBucket pull request 177](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/177)
    1. Added a function to calculate connected components in undirected
       graphs:
    * [BitBucket pull request 190](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/190)
    1. Improved the performance of `graph::InDegree()` and `graph::IncidentsTo()`.
    * [BitBucket pull request 188](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/188)
    * [Issue 79](https://github.com/ignitionrobotics/ign-math/issues/79)

1. Added Inline Versioned Namespace
    * [BitBucket pull request 216](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/216/)

## Ignition Math 3.x

### Ignition Math 3.x.x



### Ignition Math 3.3.0 (2017-11-27)

1. Fixed frustum falsely saying it contained AABB in some cases
    * [BitBucket pull request 193](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/193)
    * [Issue 78](https://github.com/ignitionrobotics/ign-math/issues/78)

1. Create consistent bracket operators across all Vector# types
    * [BitBucket pull request 181](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/181)

1. Change name to the generic BUILDING_DLL macro to avoid conflicts
    * [BitBucket pull request 173](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/173)

1. Fix some compiler warnings
    * [BitBucket pull request 196](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/196)

1. Suppress gtest warnings
    * [BitBucket pull request 199](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/199)

1. Move private headers to src folder
    * [BitBucket pull request 198](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/198)

1. Update configure.bat
    * [BitBucket pull request 206](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/206)

### Ignition Math 3.2.0 (2017-05-15)

1. Construct on first use in Rand class
    * [BitBucket pull request 165](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/165)

1. Extended Spline API: derivative interpolation, arc length calculation
   and tangent forcing.
    * [BitBucket pull request 162](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/162)

### Ignition Math 3.1.0 (2017-04-11)

1. Added signum functions to Helpers.hh.
    * Contribution from Martin Pecka
    * [BitBucket pull request 153](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/153)

### Ignition Math 3.0.0 (2017-01-05)

1. Deprecate many IGN_* macros in favor of static const variables in Helpers.hh
    * [BitBucket pull request 138](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/138)
    * [BitBucket pull request 137](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/137)

1. Removed exceptions. Return values should be evaluated to determine if
   errors have occured.
    * [BitBucket pull request 132](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/132)

1. Added `operator=(const Quaternion<T> &_q)` to `Matrix3`.
    * [BitBucket pull request 111](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/111)

1. Fix xenial cppcheck
    * [BitBucket pull request xxx](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/xxx)

1. Require cmake 2.8.12
    * [BitBucket pull request 76](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/76)

1. Migrate to relocatable CMake package.
   Contribution from Silvio Traversaro.
    * [BitBucket pull request 67](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/67)

1. Fix logic of installation of CMake configuration files in Windows.
   Contribution from Silvio Traversaro.
    * [BitBucket pull request 63](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/63)

## Ignition Math 2.x



## Ignition Math 2.9 (2017-11-22)

1. Fixed frustum falsely saying it contained AABB in some cases
    * [BitBucket pull request 193](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/193)

1. Added Color
    * [BitBucket pull request 150](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/150)

1. Backport updated configure.bat to ign-math2 and fix cppcheck warnings
    * [BitBucket pull request 207](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/207)

### Ignition Math 2.8

### Ignition Math 2.8.0

1. Added OrientedBox
    * [BitBucket pull request 146](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/146)

1. Added an assignment operator to the Frustum class.
    * [BitBucket pull request 144](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/144)

### Ignition Math 2.7

### Ignition Math 2.7.0

1. Add static const variables as alternative to macros in Helpers.hh
    * [BitBucket pull request 137](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/137)

1. Add new methods for floating numbers: lessOrEqual and greaterOrEqual
    * [BitBucket pull request 134](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/134)

### Ignition Math 2.6

### Ignition Math 2.6.0

1. Added copy constructor, equality operators and assignment operators to
    SphericalCoordinates class.
    * [BitBucket pull request 131](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/131)

1. Fix Euler angle conversion of quaternions near singularities
    * [BitBucket pull request 129](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/129)

1. Backport triangle3, helper functions, equality helper to work with 387 fp unit
   (Contribution from Rich Mattes).
    * [BitBucket pull request 125](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/125)
    * [BitBucket pull request 58](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/58)
    * [BitBucket pull request 56](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/56)

1. Added Matrix4<T>::LookAt
    * [BitBucket pull request 124](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/124)

1. Set Inertial Rotations
    * [BitBucket pull request 121](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/121)

1. Added SemanticVersion class
    * [BitBucket pull request 120](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/120)

### Ignition Math 2.5

### Ignition Math 2.5.0

1. Added PID class
    * [BitBucket pull request 117](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/117)

1. Added SphericalCoordinate class
    * [BitBucket pull request 108](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/108)

### Ignition Math 2.4

#### Ignition Math 2.4.1

1. Combine inertial properties of different objects, returning the equivalent
   inertial properties as if the objects were welded together.
    * [BitBucket pull request 115](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/115)

#### Ignition Math 2.4.0

1. New MassMatrix3 class
    * [BitBucket pull request 112](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/112)
1. MassMatrix3 helper functions
    * [BitBucket pull request 110](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/110)
1. Added Temperature class
    * A contribution from Shintaro Noda
    * [BitBucket pull request 113](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/113)

### Ignition Math 2.3.0

1. Added simple volumes formulas
    * [BitBucket pull request 84](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/84)
1. Add Length and SquaredLength for Vector2 with test
    * [BitBucket pull request 73](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/73)
1. Add Equal function with numerical tolerance argument
    * [BitBucket pull request 75](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/75)
1. First part of MassMatrix3 class, mostly accessors and modifiers
    * [BitBucket pull request 77](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/77)
1. Add Transpose methods for Matrix3,4 with test
    * [BitBucket pull request 74](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/74)
1. Multiplication improvements for Vector/Matrix classes
    * [BitBucket pull request 69](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/69)
1. Scalar +,- operators for Vector[234]
    * [BitBucket pull request 71](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/71)
1. Add Determinant method for Matrix[34]
    * [BitBucket pull request 72](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/72)
1. Fixes for compiling and running tests on Windows 7/Visual Studio 2013
   Contribution from Silvio Traversaro.
    * [BitBucket pull request 62](https://osrf-migration.github.io/ignition-gh-pages/#!/ignitionrobotics/ign-math/pull-requests/62)
