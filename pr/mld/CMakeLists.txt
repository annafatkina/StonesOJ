cmake_minimum_required(VERSION 3.8)
project(mld)

find_package(OpenCV REQUIRED)


set(CMAKE_CXX_STANDARD 11)
include_directories(DynKnn4/ANN)
include_directories(DynKnn4/)
set(SOURCE_FILES
        DynKnn4/ANN/ANn.cpp
        DynKnn4/ANN/ANn.h
        DynKnn4/ANN/Annperf.h
        DynKnn4/ANN/ANNx.h
        DynKnn4/ANN/bd_fix_rad_search.cpp
        DynKnn4/ANN/bd_pr_search.cpp
        DynKnn4/ANN/bd_search.cpp
        DynKnn4/ANN/bd_tree.cpp
        DynKnn4/ANN/bd_tree.h
        DynKnn4/ANN/brute.cpp
        DynKnn4/ANN/kd_dump.cpp
        DynKnn4/ANN/kd_fix_rad_search.cpp
        DynKnn4/ANN/kd_fix_rad_search.h
        DynKnn4/ANN/kd_pr_search.cpp
        DynKnn4/ANN/kd_pr_search.h
        DynKnn4/ANN/kd_search.cpp
        DynKnn4/ANN/kd_search.h
        DynKnn4/ANN/kd_split.cpp
        DynKnn4/ANN/kd_split.h
        DynKnn4/ANN/kd_tree.cpp
        DynKnn4/ANN/kd_tree.h
        DynKnn4/ANN/kd_util.cpp
        DynKnn4/ANN/kd_util.h
        DynKnn4/ANN/perf.cpp
        DynKnn4/ANN/pr_queue.h
        DynKnn4/ANN/pr_queue_k.h
        DynKnn4/ANN/stdafx.cpp
        DynKnn4/ANN/targetver.h
        DynKnn4/BasicMath.h
        DynKnn4/dynamical.cpp
        DynKnn4/dynamical.h
        DynKnn4/DynKnn.cpp
        DynKnn4/DynKnn.h
        DynKnn4/DynKnn4.cpp
        DynKnn4/DynKnn4.h
        DynKnn4/interfaceKNNDynamic.cpp
        DynKnn4/interfaceKNNDynamic.h
        DynKnn4/mymath.cpp
        DynKnn4/mymath.h)

#add_executable(mld ${SOURCE_FILES})
add_library(mld SHARED ${SOURCE_FILES})
target_link_libraries(mld PRIVATE ${OpenCV_LIBS})
