cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-math6 VERSION 6.15.1)

#============================================================================
# Find ignition-cmake
#============================================================================
# If you get an error at this line, you need to install ignition-cmake
find_package(ignition-cmake2 2.13.0 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set (c++standard 17)
set (CMAKE_CXX_STANDARD 17)
ign_configure_project(
  REPLACE_IGNITION_INCLUDE_PATH gz/math
  VERSION_SUFFIX
)

#============================================================================
# Set project-specific options
#============================================================================

option(SKIP_SWIG
      "Skip generating ruby bindings via Swig"
      OFF)

set(skip_pybind11_default_value OFF)
if (MSVC)
  set(skip_pybind11_default_value ON)
endif()

option(SKIP_PYBIND11
      "Skip generating Python bindings via pybind11"
      ${skip_pybind11_default_value})

include(CMakeDependentOption)
cmake_dependent_option(USE_SYSTEM_PATHS_FOR_RUBY_INSTALLATION
      "Install ruby modules in standard system paths in the system"
      OFF "NOT SKIP_SWIG" OFF)

cmake_dependent_option(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION
      "Install python modules in standard system paths in the system"
      OFF "NOT SKIP_PYBIND11" OFF)

cmake_dependent_option(USE_DIST_PACKAGES_FOR_PYTHON
      "Use dist-packages instead of site-package to install python modules"
      OFF "NOT SKIP_PYBIND11" OFF)

#============================================================================
# Search for project-specific dependencies
#============================================================================

#--------------------------------------
# Find eigen3
ign_find_package(
  EIGEN3
  REQUIRED_BY eigen3
  PRETTY eigen3
  PURPOSE "Provide conversions to eigen3 types")

########################################
# Include swig
if (SKIP_SWIG)
  message(STATUS "SKIP_SWIG set - disabling SWIG Ruby support")
else()
  find_package(SWIG QUIET)
  if (NOT SWIG_FOUND)
    IGN_BUILD_WARNING("Swig is missing: Language interfaces are disabled.")
    message (STATUS "Searching for swig - not found.")
  else()
    message (STATUS "Searching for swig - found version ${SWIG_VERSION}.")
  endif()

  # Include other languages if swig was found
  if (SWIG_FOUND)
    ########################################
    # Include ruby
    find_package(Ruby 1.9 QUIET)
    if (NOT RUBY_FOUND)
      IGN_BUILD_WARNING("Ruby is missing: Install ruby-dev to enable ruby interface to ignition math.")
      message (STATUS "Searching for Ruby - not found.")
    else()
      message (STATUS "Searching for Ruby - found version ${RUBY_VERSION}.")
    endif()
  endif()
endif()

########################################
# Python bindings
if (SKIP_PYBIND11)
  message(STATUS "SKIP_PYBIND11 set - disabling python bindings")
else()
  #include(IgnPython) TODO: allow to specify for what it should search and then
  # the code below can be removed; e.g. pybind needs Interpreter and Development components
  # see https://pybind11.readthedocs.io/en/stable/cmake/index.html#new-findpython-mode
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    set(IGN_PYTHON_VERSION "3")
    find_package(PythonInterp ${IGN_PYTHON_VERSION} QUIET)
    if(PYTHONINTERP_FOUND)
      set(Python3_FOUND ${PYTHONINTERP_FOUND})
      set(Python3_Interpreter_FOUND ${PYTHONINTERP_FOUND})
      set(Python3_EXECUTABLE ${PYTHON_EXECUTABLE})
      find_package(PythonLibs QUIET)
      # we found the interpreter but did we also find the libs? both are required
      set(Python3_FOUND ${PYTHONLIBS_FOUND})
      set(Python3_VERSION ${PYTHONLIBS_VERSION_STRING})
    endif()
  else()
    find_package(Python3 QUIET COMPONENTS Interpreter Development)
  endif()

  if (NOT Python3_FOUND)
    IGN_BUILD_WARNING("Python3 is missing: Python interfaces are disabled.")
    message (STATUS "Searching for Python3 - not found.")
  else()
    message (STATUS "Searching for Python3 - found version ${Python3_VERSION}.")

    set(PYBIND11_PYTHON_VERSION 3)
    find_package(pybind11 2.2 QUIET)

    if (${pybind11_FOUND})
      message (STATUS "Searching for pybind11 - found version ${pybind11_VERSION}.")
    else()
      IGN_BUILD_WARNING("pybind11 is missing: Python interfaces are disabled.")
      message (STATUS "Searching for pybind11 - not found.")
    endif()
  endif()
endif()

# Location of "fake install folder" used in tests
# Defined here at root scope so it is available for tests in src and test folders
set(FAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/fake/install")

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS eigen3)


#============================================================================
# Create package information
#============================================================================
ign_create_packages()

#============================================================================
# Configure documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md")
