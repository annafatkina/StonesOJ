cmake_minimum_required(VERSION 3.5)

project(pr)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        PosedImgs.h
        Orientation.h
        HelperFunctions.h
        HelperFunctions.cpp
        P3d.h
        PolarPoint.h
        StoneContourPlane.h
        Stone3d.h
        Stone3d.cpp
        water.cpp
        )
add_executable(pr ${SOURCE_FILES} )
target_link_libraries(pr PRIVATE ${OpenCV_LIBS})
