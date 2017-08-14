set(TEST_TYPE "PERFORMANCE")

set(tests
  graph.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

ign_build_tests(${tests})
