project (undistorted)
cmake_minimum_required(VERSION 3.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Ofast -march=native -ffast-math -m64 -flto")

set(SRCFILES
	ami_image_draw/image_draw.h
	ami_filters/filters.h
	ami_image/io_bmp/ami_bmp.h
	ami_image/image.h
	ami_lens_distortion/lens_distortion_procedures.h
	ami_lens_distortion/lens_distortion_model.h
	ami_lens_distortion/lens_distortion.h
	ami_pol/ami_pol.h
	ami_primitives/line_extraction.h
	ami_primitives/line_points.h
	ami_primitives/image_primitives.h
	ami_primitives/line.h
	ami_primitives/point2d.h
	ami_primitives/subpixel_image_contours.h
	ami_utilities/utilities.h
	
	undistorted.cpp
	
	ami_image/io_bmp/ami_bmp.cpp
	ami_lens_distortion/lens_distortion_procedures.cpp
	ami_lens_distortion/lens_distortion.cpp
	ami_lens_distortion/lens_distortion_model.cpp
	ami_pol/ami_pol.cpp
	ami_primitives/subpixel_image_contours.cpp
	ami_primitives/line_points.cpp
	ami_primitives/line_extraction.cpp
	ami_utilities/utilities.cpp
)

#OpenCV
find_package( OpenCV REQUIRED )
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV NOT FOUND")
  return()
endif()

add_executable (undistorted ${SRCFILES})
target_link_libraries(undistorted ${OpenCV_LIBS})
