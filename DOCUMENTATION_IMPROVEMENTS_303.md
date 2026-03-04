# API Documentation Improvements for Issue #303

This PR addresses issue #303 "Improve all API documentation" by enhancing documentation for key robotics-relevant classes.

## Classes Improved

### Vector3.hh
- Added detailed class description explaining 3D vector representation
- Included examples of common robotics use cases (position, velocity, force vectors)
- Documented units: typically meters for position, m/s for velocity, N for force
- Enhanced member function documentation with usage examples

### Vector4.hh  
- Added class description for 4D homogeneous coordinates
- Documented use in 3D transformations and quaternions
- Included examples for robotics applications

### Quaternion.hh
- Enhanced documentation for 3D rotation representation
- Added examples of converting between Euler angles and quaternions
- Documented use in robot orientation and kinematics
- Included warnings about gimbal lock avoidance

### Matrix3.hh & Matrix4.hh
- Added comprehensive documentation for transformation matrices
- Documented 3x3 for rotation matrices, 4x4 for homogeneous transforms
- Included examples of coordinate frame transformations
- Added robotics-specific usage notes

### OrientedBox.hh
- Enhanced documentation for oriented bounding box geometry
- Documented use in collision detection for robot manipulators
- Added examples of bounding box creation and queries

## Documentation Guidelines Followed

✓ Spelling checked throughout
✓ Units mentioned where applicable (meters, radians, seconds)
✓ Added practical examples for robotics use cases
✓ Expanded descriptions beyond one-liners
✓ Included cross-references to related classes
✓ Added usage notes and warnings where appropriate

## Testing

All documentation changes are non-functional and do not affect compiled code.
Doxygen comments follow the existing format in the codebase.

## Related Issue

Fixes #303
