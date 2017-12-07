cmake_minimum_required(VERSION 3.8)

#include("cmake/HunterGate.cmake")
#HunterGate(
#        URL "https://github.com/ruslo/hunter/archive/v0.19.144.tar.gz"
#        SHA1 "e3efd470f8b60468db8730eac8c04792f77fe1b1"
#)

project(pr)
include_directories(/home/anna/oj/mld/DynKnn4/)
include_directories(/home/anna/oj/mld/DynKnn4/ANN/)
#include_directories(/home/anna/oj/CGAL-4.11/build/include/)
#hunter_add_package(OpenCV)
#find_package (CGAL REQUIRED)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

add_subdirectory(mld/)

set(SOURCE_FILES water.cpp)
add_executable(pr ${SOURCE_FILES} )
target_link_libraries(pr PRIVATE ${OpenCV_LIBS} mld)
