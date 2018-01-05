cmake_minimum_required(VERSION 3.8)

project(pr)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES water.cpp)
add_executable(pr ${SOURCE_FILES} )
target_link_libraries(pr PRIVATE ${OpenCV_LIBS})
