include (${project_cmake_dir}/Utils.cmake)

set (headers
  Edge.hh
  Graph.hh
  Vertex.hh
)

set (ign_headers "" CACHE INTERNAL "Ignition math headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(ign_graph_headers
    "Ignition math headers" "#include <ignition/math/graph/${hdr}>\n")
endforeach()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/graph.hh.in
               ${CMAKE_CURRENT_BINARY_DIR}/graph.hh)
ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition/math" ${CMAKE_CURRENT_BINARY_DIR}/graph.hh)

ign_install_includes("${INCLUDE_INSTALL_DIR_POSTFIX}/ignition/${IGN_PROJECT_NAME}/graph" ${headers})
