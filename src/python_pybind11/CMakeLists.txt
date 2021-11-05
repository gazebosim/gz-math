find_package(pybind11 REQUIRED)

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()


function(clean_windows_flags target)
  # Hack to avoid pybind11 issue.
  #
  # TODO(ivanpauno):
  # This can be deleted when we update `pybind11_vendor` to a version including
  # https://github.com/pybind/pybind11/pull/2590.
  #
  # They are enabling /LTCG on Windows to reduce binary size,
  # but that doesn't play well with MSVC incremental linking (default for Debug/RelWithDebInfo).
  #
  # See:
  # - https://docs.microsoft.com/en-us/cpp/build/reference/incremental-link-incrementally?view=vs-2019
  # - https://docs.microsoft.com/en-us/cpp/build/reference/ltcg-link-time-code-generation?view=vs-2019

  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    get_target_property(target_link_libraries ${target} LINK_LIBRARIES)
    list(REMOVE_ITEM target_link_libraries "$<$<NOT:$<CONFIG:Debug>>:-LTCG>")
    set_target_properties(${target} PROPERTIES LINK_LIBRARIES "${target_link_libraries}")

    get_target_property(target_compile_options ${target} COMPILE_OPTIONS)
    list(REMOVE_ITEM target_compile_options "$<$<NOT:$<CONFIG:Debug>>:/GL>")
    set_target_properties(${target} PROPERTIES COMPILE_OPTIONS "${target_compile_options}")
  endif()
endfunction()

# # enables using the Python extensions from the build space for testing
# file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_rclpy/__init__.py" "")

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install into test_rclpy folder in build space for unit tests to import
  set_target_properties(${_library_name} PROPERTIES
    # Use generator expression to avoid prepending a build type specific directory on Windows
    LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>
    RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>)

  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()


# Split from main extension and converted to pybind11
pybind11_add_module(ignition_math SHARED
  src/_ignition_math_pybind11.cc
)
