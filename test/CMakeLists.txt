configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)

add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
