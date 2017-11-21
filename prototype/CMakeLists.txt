cmake_minimum_required(VERSION 3.5)
project(prototype)

set(CMAKE_CXX_STANDARD 11)
set (CMAKE_SHARED_LINKER_FLAGS "")

set(SOURCE_FILES
    src/main_video.cpp)

add_executable(prototype ${SOURCE_FILES})

#setting up OpenCV
find_package( OpenCV REQUIRED )
target_link_libraries( prototype ${OpenCV_LIBS} )

# try the shared tensorflow library first
find_package(TensorflowCC COMPONENTS Shared)
if(TensorflowCC_Shared_FOUND)
  target_link_libraries(prototype TensorflowCC::Shared)
# fallback to the static library
else()
  find_package(TensorflowCC REQUIRED COMPONENTS Static)
  target_link_libraries(example TensorflowCC::Static)
endif()


