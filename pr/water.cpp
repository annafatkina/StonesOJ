#define _USE_MATH_DEFINES

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "HelperFunctions.h"
#include "Stone3d.h"
#include "StoneContourPlane.h"
#include "PolarPoint.h"
#include "PosedImgs.h"

using namespace std;
using namespace cv;

Mat colorchanging(Mat gray, Mat real_conts) {
	if (!gray.data || !real_conts.data) {
		std::cout << "Nodata";
	}
//	imshow("Red Result BF", gray*10000);
	Mat mask; 
	gray *= 100;
	gray.convertTo(mask, real_conts.type());
	Mat mask2 = real_conts.clone();
	cvtColor(mask2, mask2, CV_BGR2GRAY);
	//bitwise_not(mask2, mask2);
	mask.copyTo(gray, mask2);
	Mat res = Mat::zeros(gray.size(), CV_32FC3);
	res.setTo(Scalar(255,255,255));
	res.setTo(Scalar(0, 0, 255), mask);
	res.setTo(Scalar(0, 0, 0), mask2);
	imshow("Red Result", res);
	return res;
}

Mat img_cutter(Mat input_mat) {  //, vector<Mat> &output_vec) {
	vector<vector<Point>> contours = extractContFromImg(input_mat);
	Mat markers = Mat::zeros(input_mat.size(), CV_32SC1);
	int conts_size = contours.size();

	//for (int i = 0; i < conts_size; i++) {
		drawContours(markers, contours, static_cast<int>(0),
			     Scalar::all(static_cast<int>(0) + 1), -1);
	//	imshow("cutter beforeooooo", markers);
	//	markers *= 100;
		markers = colorchanging(markers, input_mat);

		Mat m2;
		imshow("cutter before", markers);
		//std::cout << "mat " << std::endl << markers << std::endl; 
		//markers /= 180;
	//	markers.convertTo(markers, CV_32F);
	//	markers /=1000.0;
	//	imshow("cutter after", markers);
		
		//std::cout << "mat " << std::endl << markers << std::endl; 
	//	cvtColor(markers, m2, CV_GRAY2BGR, 3);
		std::cout << "mark type = " << markers.type()
			  << " m2 type = " << m2.type();

		//m2.setTo (Scalar(255, 0, 0), markers);
	//	std::cout << "mat " << std::endl << m2 << std::endl; 
		m2 = markers;
		if (!m2.data) {
			std::cout << "Nodata";
		}
		vector<vector<Point>> new_contours = extractContFromImg(m2);
		conts_size = contours.size();

		/*for (int i = 0; i < conts_size; i++) {
			drawContours(m2, new_contours, static_cast<int>(i),
				     Scalar::all(static_cast<int>(i) + 1), -1);
		}*/
	std::cout << " src.cont.size = " << conts_size;	
		imshow("cutter", m2);
	//}
	Mat res;
	m2.convertTo(res, CV_8UC3);
	std::cout << "m2 size: " << m2.cols  << " " << m2.rows << " " 
		<< std::endl << "res size " << res.cols << " " << res.rows << std::endl;
//	waitKey();
	return res;
}

void devider(vector<Mat> mats);

void combineImgs(vector<PosedImgs> imgs) {
	int imgscount = imgs.size();
	shared_ptr<vector<Stone3d<double>>> stone3dVecPtr =
	    make_shared<vector<Stone3d<double>>>();
	for (int k = 0; k < imgscount; k++) {
		Mat src(imgs[k].getMat());
		std::cout << "combine type = " << src.type() << " " ;
	src = img_cutter(src);
	vector<Mat> mats;
	mats.push_back(src);
	devider(mats);	
	waitKey();
		vector<vector<Point>> contours = extractContFromImg(src);
		std::cout << "combine cont size = " << contours.size() << " ";  
		Mat markers = Mat::zeros(src.size(), CV_32SC1);
		Mat squares = Mat::zeros(contours.size(), 1, CV_64F);
		Mat markers_tmp;
		int conts_size = contours.size();
		for (int i = 0; i < conts_size; i++) {
			markers = Mat::zeros(src.size(), CV_32SC1);
			drawContours(markers, contours, static_cast<int>(i),
				     Scalar::all(static_cast<int>(i) + 1), -1);
			squares.at<double>(i) = (double)findSq(markers, 0);
			std::cout << "squares.at<double>(i)  = "
				  << squares.at<double>(i) << std::endl;
			double r = sqrt(squares.at<double>(i) * M_1_PI);
			Point tmp = findContCenter(contours[i]);
			markers_tmp = markers.clone();
			Point center = findContCenter(contours[i]);
			contours[i] = makefullcont(contours[i], 5);
			int ttttmp = 0;
			vector<PolarPoint<double>> difs = compareWithCircle(
			    markers_tmp, contours[i], r, center, ttttmp);
			recoverStone(difs, center, r, markers_tmp);

			shared_ptr<vector<double>> outptr =
			    make_shared<vector<double>>();
			int si = extractRFromPP(outptr, difs);

			imshow("not resized" + to_string(i),
				   markers_tmp * 10000);

			vector<double>& outvec = *outptr;
			int outs = si;
			StoneContourPlane important;
			important.orient = imgs[k].orient;
			important.xShift = imgs[k].beginX;
			important.yShift = imgs[k].beginY;
			important.zShift = imgs[k].beginZ;
			important.contour = contours[i];
			addToStones(important, stone3dVecPtr, r);
		}
	}
	vector<Stone3d<double>>& stone3dVec = *stone3dVecPtr;
	int st3dsize = stone3dVec.size();
	for (int st = 0; st < st3dsize; st++) {
		stone3dVec[st].makeDense(5);
	}
	for (int st = 0; st < st3dsize; st++) {
		stone3dVec[st].toFile("ooooooout" + to_string(st) + ".xyz");
	}
	std::cout << "Im finished!" << std::endl;
}

void devider( vector<Mat> input_imgs) {//vector<PosedImgs> input_imgs) {
	vector<PosedImgs> returnable = {};
	size_t pim_size = input_imgs.size();
	for (size_t pim_i = 0; pim_i < pim_size; pim_i++) {// auto src : input_imgs) {
//		Mat src = input_imgs[pim_i].img;
		Mat src = input_imgs[pim_i]; 
		for (int x = 0; x < src.rows; x++) {
			for (int y = 0; y < src.cols; y++) {
				if (src.at<Vec3b>(x, y) ==
				    Vec3b(255, 255, 255)) {
					src.at<Vec3b>(x, y)[0] = 0;
					src.at<Vec3b>(x, y)[1] = 0;
					src.at<Vec3b>(x, y)[2] = 0;
				}
			}
		}
		// colorchanging(src);
		// Show output image
		imshow("Black Background Image", src);
		// Create a kernel that we will use for accuting/sharpening our
		// image
		Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1,
			      1);  // an approximation of second derivative, a
				   // quite strong kernel
		// do the laplacian filtering as it is
		// well, we need to convert everything in something more deeper
		// then CV_8U
		// because the kernel has some negative values,
		// and we can expect in general to have a Laplacian image with
		// negative values
		// BUT a 8bits unsigned int (the one we are working with) can
		// contain values from 0 to 255
		// so the possible negative number will be truncated
		Mat imgLaplacian;
		Mat sharp = src;  // copy source image to another temporary one
		filter2D(sharp, imgLaplacian, CV_32F, kernel);
		src.convertTo(sharp, CV_32F);
		Mat imgResult = sharp - imgLaplacian;
		// convert back to 8bits gray scale
		imgResult.convertTo(imgResult, CV_8UC3);
		imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
		// imshow( "Laplace Filtered Image", imgLaplacian );
		imshow("New Sharped Image", imgResult);
		// Create binary image from source image
		Mat bw;
		cvtColor(src, bw, CV_BGR2GRAY);
		threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
		imshow("Binary Image", bw);
		// Perform the distance transform algorithm
		Mat dist;
		distanceTransform(bw, dist, CV_DIST_L2, 3);
		// Normalize the distance image for range = {0.0, 1.0}
		// so we can visualize and threshold it
		normalize(dist, dist, 0, 1., NORM_MINMAX);
		imshow("Distance Transform Image", dist);
		// Threshold to obtain the peaks
		// This will be the markers for the foreground objects
		threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
		// Dilate a bit the dist image
		Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
		dilate(dist, dist, kernel1);
		imshow("Peaks", dist);
		// Create the CV_8U version of the distance image
		// It is needed for findContours()
		Mat dist_8u;
		dist.convertTo(dist_8u, CV_8U);
		// Find total markers
		vector<vector<Point>> contours;
		findContours(dist_8u, contours, CV_RETR_EXTERNAL,
			     CV_CHAIN_APPROX_SIMPLE);
		// Create the marker image for the watershed algorithm
		Mat markers = Mat::zeros(dist.size(), CV_32SC1);
		// Draw the foreground markers
		for (size_t i = 0; i < contours.size(); i++)
			drawContours(markers, contours, static_cast<int>(i),
				     Scalar::all(static_cast<int>(i) + 1), -1);
		// Draw the background marker
		circle(markers, Point(5, 5), 3, CV_RGB(255, 255, 255), -1);
		imshow("Markers", markers * 10000);
		// Perform the watershed algorithm
		watershed(src, markers);
		Mat mark = Mat::zeros(markers.size(), CV_8UC1);
		markers.convertTo(mark, CV_8UC1);
		bitwise_not(mark, mark);
		//    imshow("Markers_v2", mark); // uncomment this if you want
	//    to see how the mark
		// image looks like at that point
		// Generate random colors
		vector<Vec3b> colors;
		for (size_t i = 0; i < contours.size(); i++) {
			int b = theRNG().uniform(0, 255);
			int g = theRNG().uniform(0, 255);
			int r = theRNG().uniform(0, 255);
			colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
		}
		// Create the result image
		Mat dst = Mat::zeros(markers.size(), CV_8UC3);
		std::cout << "divider cont size = " << contours.size() << " "; 
		// Fill labeled objects with random colors
		for (int i = 0; i < markers.rows; i++) {
			for (int j = 0; j < markers.cols; j++) {
				int index = markers.at<int>(i, j);
				if (index > 0 &&
				    index <= static_cast<int>(contours.size()))
					dst.at<Vec3b>(i, j) = colors[index - 1];
				else
					dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
		for (int i =0 ; i < contours.size(); i++) {
			std::cout << "cont points : " << contours[i].size() << " ";
			drawContours(dst, contours, static_cast<int>(i),
                                     Scalar::all(static_cast<int>(i) + 1), -1);
		}
		// Visualize the final image
		imshow("Final Result", dst);
	}
	//return returnable;
}

int main(int, char** argv) {
	// Mat src = imread(argv[1]);
	Mat front1 = imread(argv[1]);
	/*	if (!front1.data) {
			std::cout << "First err!";
			exit(-1);
		}*/
	Mat front2 = imread(argv[2]);
	// Mat front3 = imread(argv[3]);
	float scale = atof(argv[3]);
	Mat front4 = imread("../example.png");
	vector<PosedImgs> sources = {};
	PosedImgs mat1(front4, xOrient);
	// sources.push_back(mat1);
	int tmp1 = front1.rows / 2;
	int tmp2 = front2.rows / 2;
	vector<Mat> tMat = {};
	tMat.push_back(front4);
	//	devider(tMat);
	//       std::cout << "sizes: " << front1.rows << " " << front2.cols <<
	//       std::endl;
	/*	PosedImgs mat0(front1, xOrient, tmp1,  0, 0, scale);// 500, 0,
	   0);
		PosedImgs mat2(front2, yOrient, 0, tmp2, 0, scale); //-1200,
	   1700, 0);
		sources.push_back(mat0);
		sources.push_back(mat2);
	*/
	sources.push_back(mat1);
	combineImgs(sources);
	std::cout << "Im in the end of main" << std::endl;
	waitKey(0);
	return 0;
}