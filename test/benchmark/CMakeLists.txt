set(TEST_TYPE "BENCHMARK")

# Skip silently when google-benchmark (libbenchmark-dev) isn't installed:
# calling gz_add_benchmarks emits a CMake warning that some CI pipelines
# treat as a build issue, and the benchmark binaries are strictly optional
# anyway.
find_package(benchmark QUIET)
if (NOT benchmark_FOUND)
  return()
endif()

include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)
if (GzBenchmark_FOUND)
  set(tests
    graph.cc
    gz_sim_workload.cc
  )

  gz_add_benchmarks(SOURCES ${tests})
endif()
