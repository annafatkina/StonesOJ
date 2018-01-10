cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2 CXX)

function(assign_source_group)
    foreach(_SOURCE IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_SOURCE}")
            file(RELATIVE_PATH _SOURCE_REL "${CMAKE_CURRENT_SOURCE_DIR}" "${_SOURCE}")
        else()
            set(_SOURCE_REL "${_SOURCE}")
        endif()
        get_filename_component(_SOURCE_PATH "${_SOURCE_REL}" PATH)
        if(WIN32)
            string(REPLACE "/" "\\" _SOURCE_PATH_MSVC "${_SOURCE_PATH}")
            source_group("${_SOURCE_PATH_MSVC}" FILES "${_SOURCE}")
        else()
            source_group("${_SOURCE_PATH}" FILES "${_SOURCE}")
        endif()
    endforeach()
endfunction(assign_source_group)


IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  ") # -Ofast -m64 -flto  -march=native
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   ") # -Ofast -m64 -flto  -march=native -march=native

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(GLOB_RECURSE SOURCES
        "src/*"
        "include/*"
        )

set(SOURCE_FILES ${SOURCES})
assign_source_group(${SOURCE_FILES})
include_directories("Thirdparty/DBoW2/") #TODO: make a setable parameter with default value
include_directories("Thirdparty/g2o/")
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

#OpenCV
find_package( OpenCV REQUIRED )
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV NOT FOUND")
  return()
endif()

#PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(slam slam.cpp)
target_link_libraries(slam ${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})
