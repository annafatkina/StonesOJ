cmake_minimum_required(VERSION 3.5)
project(prototype)

set(CMAKE_CXX_STANDARD 11)
set (CMAKE_SHARED_LINKER_FLAGS "")

set(SOURCE_FILES
    src/main_video.cpp)

add_executable(prototype ${SOURCE_FILES})

#setting up OpenCV
set (OpenCV_DIR ${PROJECT_SOURCE_DIR}/install/opencv/share/OpenCV)
find_package( OpenCV REQUIRED )
target_link_libraries( prototype ${OpenCV_LIBS} )

#setting up Tensorflow
set (TENSORFLOW_DIR ${PROJECT_SOURCE_DIR}/install/tensorflow)
include_directories(
        "${TENSORFLOW_DIR}/include/tensorflow"
        "${TENSORFLOW_DIR}/include/tensorflow/bazel-genfiles"
        "${TENSORFLOW_DIR}/include/tensorflow/tensorflow/contrib/makefile/downloads"
        "${TENSORFLOW_DIR}/include/tensorflow/tensorflow/contrib/makefile/downloads/eigen"
        "${TENSORFLOW_DIR}/include/tensorflow/tensorflow/contrib/makefile/downloads/gemmlowp"
        "${TENSORFLOW_DIR}/include/tensorflow/tensorflow/contrib/makefile/downloads/nsync/public"
        "${TENSORFLOW_DIR}/include/tensorflow/tensorflow/contrib/makefile/gen/protobuf-host/include")

target_link_libraries(prototype
        "${TENSORFLOW_DIR}/lib/tensorflow_cc/libtensorflow_framework.so"
        "${TENSORFLOW_DIR}/lib/tensorflow_cc/libtensorflow_cc.so"
        "${TENSORFLOW_DIR}/lib/tensorflow_cc/libprotobuf.a"
        dl pthread)
