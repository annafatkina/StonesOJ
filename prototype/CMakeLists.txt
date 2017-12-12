project (prototype)
cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES prototype.cpp)

#OpenCV
find_package( OpenCV REQUIRED )
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV NOT FOUND")
  return()
endif()

#Tensorflow
find_package(TensorflowCC COMPONENTS Shared)
if(NOT TensorflowCC_Shared_FOUND)
  message(STATUS "Tensorflow NOT FOUND")
  return()
endif()

#PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (prototype prototype.cpp)
target_link_libraries(prototype ${OpenCV_LIBS} TensorflowCC::Shared ${PCL_LIBRARIES})
