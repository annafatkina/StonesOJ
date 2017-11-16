cmake_minimum_required(VERSION 3.8)

#include("cmake/HunterGate.cmake")
#HunterGate(
#        URL "https://github.com/ruslo/hunter/archive/v0.19.144.tar.gz"
#        SHA1 "e3efd470f8b60468db8730eac8c04792f77fe1b1"
#)

project(pr)
#hunter_add_package(OpenCV)
find_package(OpenCV REQUIRED)
#include_directories(/usr/include/ImageMagick-6/)
include_directories(${ImageMagick_Magick++_INCLUDE_DIR} ${ImageMagick_Magick++_ARCH_INCLUDE_DIR})
message(STATUS "IM: " ${ImageMagick_Magick++_INCLUDE_DIR})
find_package(ImageMagick REQUIRED)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES water.cpp)
add_executable(pr ${SOURCE_FILES})
target_link_libraries(pr PRIVATE ${OpenCV_LIBS})
