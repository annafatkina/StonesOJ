/*
Copyright (c) 2010-2013, AMI RESEARCH GROUP <lalvarez@dis.ulpgc.es>
License : CC Creative Commons "Attribution-NonCommercial-ShareAlike"
see http://creativecommons.org/licenses/by-nc-sa/3.0/es/deed.en
*/


/**
* @file lens_distortion_correction_division_model_1p.cpp
* @brief distortion correction using the improved Hough transform with the 1 parameter division distortion model
*
* @author Luis Alvarez <lalvarez@dis.ulpgc.es> and Daniel Santana-Cedres <dsantana@ctim.es>
*/

// Note: for VS2015 use NuGet
// Install-Package opencvdefault 
// Install-Package opencvcontrib

//Included libraries
#include "ami_image/image.h"
#include "ami_filters/filters.h"
#include "ami_primitives/subpixel_image_contours.h"
#include "ami_primitives/line_extraction.h"
#include "ami_primitives/image_primitives.h"
#include "ami_lens_distortion/lens_distortion_procedures.h"
#include "ami_utilities/utilities.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace cv;

struct Model {
	Model() {
		image_amplification_factor = 2.0;
		error = 0;
	}
	std::vector<double> d /** RADIAL DISTORSION MODEL POLYNOMIAL */;
	double cx; /** CENTER OF THE DISTORSION MODEL */;
	double cy; /** CENTER OF THE DISTORSION MODEL */;
	double error; // model error
	double image_amplification_factor; // integer index to fix the way the corrected image is scaled to fit input size image
};

struct UndistortEngineSettings {
	UndistortEngineSettings() {
		canny_high_threshold = 0.8;
		initial_distortion_parameter=0.0;
		final_distortion_parameter = 3.0;
		distance_point_line_max_hough = 3.0;
		angle_point_orientation_max_difference = 2.0;

		max_lines=30;
		angle_resolution=0.1; 
		distance_resolution=1.; 
		distortion_parameter_resolution=0.1;
	}

	float canny_high_threshold;// high threshold for canny detector
	float initial_distortion_parameter;//left side of allowed distortion parameter interval
	float final_distortion_parameter;//Hough parameter
	float distance_point_line_max_hough;//Hough parameter
	//maximum difference allowed (in degrees) between edge and line orientation
	float angle_point_orientation_max_difference;

	int max_lines; //maximun number of lines estimated
	float angle_resolution; // angle discretization step (in degrees)
	float distance_resolution; // line distance discretization step
	float distortion_parameter_resolution;//distortion parameter discretization step

};

struct UndistortEngine {
	Model model;

	bool CreateModel(const UndistortEngineSettings & settings, const cv::Mat & frame) {
			//We read the input image and some variables are initialized
	image<unsigned char> input(frame); // input image
	imwrite("current_frame.jpg", frame);
	auto width = input.width(), height = input.height();//input image dimensions
	auto size_ = width*height; // image size
	image<unsigned char> gray(width,height,1,0);//gray-level image to call canny
	image<unsigned char> edges(width,height,1,0);//image to store edge information
	std::cout << "step 1 passed" << endl;
	//Converting the input image to gray level
	for(auto i=0; i<size_; i++)
		gray[i] = 0.3*input[i] + 0.59*input[i+size_] + 0.11*input[i+size_*2];

	//ALGORITHM STAGE 1 : Detecting edges with Canny
	float canny_low_threshold = 0.7; //default value for canny lower threshold
		auto contours=canny(gray,edges,canny_low_threshold,settings.canny_high_threshold);

	image_primitives i_primitives_quo;//object to store output edge line structure

	//we call 3D Hough line extraction
	std::cout << "step 2 passed" << endl;
	line_equation_distortion_extraction_improved_hough_quotient(
		contours,
		i_primitives_quo,
		settings.distance_point_line_max_hough,
		settings.max_lines,
		settings.angle_resolution,
		settings.distance_resolution,
		settings.initial_distortion_parameter,
		settings.final_distortion_parameter,
		settings.distortion_parameter_resolution,
		settings.angle_point_orientation_max_difference
		);

	// WE CHECK IF THE IMPROVED HOUGH PROCEDURE FINISHS PROPERLY
	if(i_primitives_quo.get_lines().size()==0){
		std::cout << "Training failed, returning false" << endl;
		return false;
	}
	std::cout << "step 3 passed" << endl;
	auto distortion = i_primitives_quo.get_distortion();

	model.error = model_estimation_1p_quotient(i_primitives_quo.get_distortion().get_distortion_center(),
		i_primitives_quo.get_lines(),
		distortion );


	model.d = distortion .get_d();
	model.cx = distortion .get_distortion_center().x;
	model.cy = distortion .get_distortion_center().y;
	std::cout << "step 4 passed" << endl;

	}

	Mat ApplyModel(cv::Mat frame) {

		image<unsigned char> input(frame); // input image

		lens_distortion_model m;
		m.set_d(model.d);
		ami::point2d<double> c;
		c.x = model.cx;
		c.y = model.cy;
		m.set_distortion_center(c);

		return undistort_quotient_image_inverse(
			input, // input image
			m, // lens distortion model
			model.image_amplification_factor // integer index to fix the way the corrected image is scaled to fit input size image
			).GetMat();
		
	}
};

void process_frame(Mat src, Mat& dst)
{
	Mat current = src.clone();
	fastNlMeansDenoisingColored(current, current);

	UndistortEngineSettings settings;
	UndistortEngine ue;
	auto train = [&]() {
		auto cl = createCLAHE();
		cvtColor(current, current, COLOR_BGR2GRAY);
		cl->apply(current, current);
		cvtColor(current, current, COLOR_GRAY2RGB);

		auto train_t = chrono::system_clock::now().time_since_epoch();
		bool trained = ue.CreateModel(settings, current);
		if(!trained) {
			cout << "training failed!" << endl;
		}
		auto train_dt = chrono::duration_cast<chrono::milliseconds >(chrono::system_clock::now().time_since_epoch() - train_t).count();
		cout << "distortion model trained in " << train_dt << "ms." <<  endl;
	};
	train();

	auto show = [&]() {
			auto t = chrono::system_clock::now().time_since_epoch();
			current = ue.ApplyModel(current);
			auto dt = chrono::duration_cast<chrono::milliseconds >(chrono::system_clock::now().time_since_epoch() - t).count();
			cout << "distortion model applied in " << dt << "ms." <<  endl;
			dst = current.clone();
	};
	show();
}

int main(int argc, char* argv[]) {

	string input_video_path, output_video_path;

	for (int i = 0; i < argc; i++) {
		if (argv[i][0] == '-' && argv[i][1] == 'i') {
			input_video_path = argv[i+1];
		}
		if (argv[i][0] == '-' && argv[i][1] == 'o') {
			output_video_path = argv[i+1];
		}
	}


	
        Mat current_image;
	auto reader = VideoCapture(input_video_path);
	
	auto codec = CV_FOURCC('X', 'V', 'I', 'D');
	auto fps = 30.0;
	Size output_video_size;
	output_video_size = Size(625, 500);
	auto writer = VideoWriter(output_video_path, codec, fps, output_video_size, true);
        
	while (reader.read(current_image)) {
		Mat undistorted_image;
		process_frame(current_image.clone(), undistorted_image);
		writer.write(undistorted_image);
	}
	writer.release();	
	return 0;
}
