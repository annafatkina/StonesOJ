cmake_minimum_required(VERSION 3.5)

project(point_cloud_stone)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES water.cpp)
add_executable(point_cloud_stone ${SOURCE_FILES} )
target_link_libraries(point_cloud_stone PRIVATE ${OpenCV_LIBS})
