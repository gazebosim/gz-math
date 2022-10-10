if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()

message(STATUS "Building pybind11 interfaces")
set(BINDINGS_MODULE_NAME "math${PROJECT_VERSION_MAJOR}")
# Split from main extension and converted to pybind11
pybind11_add_module(${BINDINGS_MODULE_NAME} MODULE
  src/_gz_math_pybind11.cc
  src/Angle.cc
  src/AxisAlignedBox.cc
  src/Capsule.cc
  src/Color.cc
  src/DiffDriveOdometry.cc
  src/Ellipsoid.cc
  src/Filter.cc
  src/Frustum.cc
  src/GaussMarkovProcess.cc
  src/Helpers.cc
  src/Interval.cc
  src/Kmeans.cc
  src/Line2.cc
  src/Line3.cc
  src/MassMatrix3.cc
  src/Material.cc
  src/Matrix3.cc
  src/Matrix4.cc
  src/Matrix6.cc
  src/MovingWindowFilter.cc
  src/PID.cc
  src/Polynomial3.cc
  src/Pose3.cc
  src/Quaternion.cc
  src/Rand.cc
  src/Region3.cc
  src/RollingMean.cc
  src/RotationSpline.cc
  src/SemanticVersion.cc
  src/SignalStats.cc
  src/SphericalCoordinates.cc
  src/Spline.cc
  src/StopWatch.cc
  src/Temperature.cc
  src/Triangle.cc
  src/Triangle3.cc
  src/Vector2.cc
  src/Vector3.cc
  src/Vector4.cc
  src/Vector3Stats.cc
)

target_link_libraries(${BINDINGS_MODULE_NAME} PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
)

target_compile_definitions(${BINDINGS_MODULE_NAME} PRIVATE
  BINDINGS_MODULE_NAME=${BINDINGS_MODULE_NAME})

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # Workaround for Clang and pybind11 on Focal
  # https://github.com/pybind/pybind11/issues/1604
  # Resolved by newer versions of pybind11
  if(${pybind11_VERSION} VERSION_LESS "2.4.4")
    target_compile_options(${BINDINGS_MODULE_NAME} PRIVATE -fsized-deallocation)
  endif()

  # Suppress warnings that clang misidentifies:
  # https://github.com/pybind/pybind11/issues/1893
  target_compile_options(${BINDINGS_MODULE_NAME} PRIVATE -Wno-self-assign-overloaded)
endif()

if(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION)
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
  from distutils import sysconfig as sc
  print(sc.get_python_lib(plat_specific=True))"
      OUTPUT_VARIABLE Python3_SITEARCH
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  else()
    # Get install variable from Python3 module
    # Python3_SITEARCH is available from 3.12 on, workaround if needed:
    find_package(Python3 COMPONENTS Interpreter)
  endif()

  if(USE_DIST_PACKAGES_FOR_PYTHON)
    string(REPLACE "site-packages" "dist-packages" GZ_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
  else()
    # custom cmake command is returning dist-packages
    string(REPLACE "dist-packages" "site-packages" GZ_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
  endif()
else()
  # If not a system installation, respect local paths
  set(GZ_PYTHON_INSTALL_PATH ${GZ_LIB_INSTALL_DIR}/python)
endif()

set(GZ_PYTHON_INSTALL_PATH "${GZ_PYTHON_INSTALL_PATH}/gz")

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install into test folder in build space for unit tests to import
  set_target_properties(${_library_name} PROPERTIES
    # Use generator expression to avoid prepending a build type specific directory on Windows
    LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test>
    RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test>)

  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${GZ_PYTHON_INSTALL_PATH}/"
  )
endfunction()

configure_build_install_location(${BINDINGS_MODULE_NAME})

if (BUILD_TESTING)
  # Add the Python tests
  set(python_tests
    Angle_TEST
    AxisAlignedBox_TEST
    Box_TEST
    Capsule_TEST
    Color_TEST
    Cylinder_TEST
    DiffDriveOdometry_TEST
    Ellipsoid_TEST
    Filter_TEST
    Frustum_TEST
    GaussMarkovProcess_TEST
    Helpers_TEST
    Inertial_TEST
    Interval_TEST
    Kmeans_TEST
    Line2_TEST
    Line3_TEST
    MassMatrix3_TEST
    Material_TEST
    Matrix3_TEST
    Matrix4_TEST
    Matrix6_TEST
    MovingWindowFilter_TEST
    OrientedBox_TEST
    PID_TEST
    Plane_TEST
    Polynomial3_TEST
    Pose3_TEST
    Quaternion_TEST
    Rand_TEST
    Region3_TEST
    RollingMean_TEST
    RotationSpline_TEST
    SemanticVersion_TEST
    SignalStats_TEST
    Sphere_TEST
    SphericalCoordinates_TEST
    Spline_TEST
    StopWatch_TEST
    Temperature_TEST
    Triangle3_TEST
    Triangle_TEST
    Vector2_TEST
    Vector3_TEST
    Vector3Stats_TEST
    Vector4_TEST
  )

  execute_process(COMMAND "${Python3_EXECUTABLE}" -m pytest --version
    OUTPUT_VARIABLE PYTEST_output
    ERROR_VARIABLE  PYTEST_error
    RESULT_VARIABLE PYTEST_result)
  if(${PYTEST_result} EQUAL 0)
    set(pytest_FOUND TRUE)
  else()
    message("")
    message(WARNING "Pytest package not available: ${PYTEST_error}")
  endif()

  foreach (test ${python_tests})
    if (pytest_FOUND)
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" -m pytest "${CMAKE_SOURCE_DIR}/src/python_pybind11/test/${test}.py" --junitxml "${CMAKE_BINARY_DIR}/test_results/${test}.xml")
    else()
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/src/python_pybind11/test/${test}.py")
    endif()

    set(_env_vars)
    list(APPEND _env_vars "PYTHONPATH=${FAKE_INSTALL_PREFIX}/lib/python/")
    list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
    set_tests_properties(${test}.py PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endforeach()

endif()
