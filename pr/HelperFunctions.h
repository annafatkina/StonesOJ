//
// Created by anna on 26.02.18.
//

#ifndef PR_HELPERFUNCTIONS_H
#define PR_HELPERFUNCTIONS_H

#include "PosedImgs.h"
#include "P3d.h"
#include "PolarPoint.h"
//#include "StoneContourPlane.h"
#include "Stone3d.h"
#include "Orientation.h"



template <typename T>
T signum(T in);

Point findContCenter(vector<Point> contour);

/**
*	Recovers contour by data of compared circle and
*	defference from this.
*/

vector<Point> recoverStone(vector<PolarPoint<double>> vectorizedStone,
Point center, double r,
        Mat recStone);

/**
*	Find an array of difference between input cintour
*	and circle radius of r with center placed in the
*	center of given contour.
*	Output also saves an information of sines and
*	cosines of every point of contour (have to be
*	saved to have a recovery oppotunity).
*/
vector<PolarPoint<double>> compareWithCircle(Mat circleImg,
                                             vector<Point> contour, double r,
                                             Point center, int& deviation);

vector<Point> makefullcont(vector<Point> in, int step = 1);

int findMaxDim(vector<Point> in);

double GetContRange(vector<P3d<double>> in, Orientation orient, Point& cent);

/**
*	Find square of figure that has color different from bgcolor
*/
int findSq(Mat markers, int bgcolor);

/**
*	Find the center of input contour
*/
P3d<double> findContCenter3dPlane(vector<P3d<double>> contour3d,
                                  Orientation orient, int step);


/**
*	Extract radiuses of points from PolarPoint array.
*	TODO: make template
*/
int extractRFromPP(shared_ptr<vector<double>> retVec,
                   vector<PolarPoint<double>> vecPP);


vector<vector<Point>> extractContFromImg(Mat src);


template <typename T>
vector<Point> make2d(vector<P3d<T>> contour3d, Orientation orient);

vector<vector<Point>> extractContFromImg(Mat src);


#endif //PR_HELPERFUNCTIONS_H
