project (mesh-saver)
cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES mesh-saver.cpp)

#Eigen
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
  include(${EIGEN3_USE_FILE})
endif()

#CGAL
find_package(CGAL QUIET COMPONENTS REQUIRED)
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
find_package(OpenMesh REQUIRED)
if (NOT OpenMesh_FOUND)
  message(STATUS "OpenMesh NOT FOUND")
  return()
endif()

add_executable (mesh-saver ${SOURCE_FILES})
add_to_cached_list ( CGAL_EXECUTABLE_TARGETS mesh-saver )
target_link_libraries(mesh-saver ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} OpenMeshCore OpenMeshTools)
