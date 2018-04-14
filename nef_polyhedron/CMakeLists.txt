project (nef_polyhedron)
cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES nef_polyhedron.cpp)

#Eigen
find_package(Eigen3)
if (EIGEN3_FOUND)
  include(${EIGEN3_USE_FILE})
endif()

#CGAL
find_package(CGAL QUIET COMPONENTS)
if (NOT CGAL_FOUND)
  message(STATUS "CGAL NOT FOUND")
  return()
endif()
include(${CGAL_USE_FILE})

#BOOST
find_package(Boost REQUIRED)
if (NOT Boost_FOUND)
  message(STATUS "Boost NOT FOUND")
  return()
endif()

#OpenMesh
find_package(OpenMesh)
if (NOT OpenMesh_FOUND)
  message(STATUS "OpenMesh NOT FOUND")
  return()
endif()

add_executable (nef_polyhedron ${SOURCE_FILES})
add_to_cached_list ( CGAL_EXECUTABLE_TARGETS nef_polyhedron )
target_link_libraries(nef_polyhedron ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} OpenMeshCore OpenMeshTools)
