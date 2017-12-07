#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <DynKnn4.h>

using namespace std;
using namespace cv;

// TODO: remake to avoid extra copies!!!
/**
*	Wrapper for for polar coordinates points 
*	Contains radius, sine(fi) and cosine(fi).
*/
template <typename T> 
struct PolarPoint {
  typedef T value_type;
  T r; 
  T rcos, rsin;

  PolarPoint() = default;
  PolarPoint(T inR, T inRcos, T inRsin) : r(inR), rcos(inRcos), rsin(inRsin) { }
  inline void printCoord() {
    std::cout << "r = " << r << ", rcos = " << rcos << ", rsin = " << rsin << std::endl;
  }
};

template <typename T>
struct P3d {
  typedef T value_type;
  P3d() {};
  P3d(T in_x, T in_y, T in_z) {
    x = in_x; y = in_y; z = in_z;
  }
  T x, y, z;
};

template<typename T>
struct StoneContourPlane {
  vector<T> contourX;
  vector<Point> contourY;
  vector<Point> contourZ;
  Point center;
  int xShift, yShift, zShift;
  StoneContourPlane() : xShift(0), yShift(0), zShift(0) {
    contourX = {};
    contourY = {};
    contourZ = {};
    center = Point(-1.0, -1.0);
  }
/*  void countCenter() {
    if (contour.size() != 0) center = findContCenter(contour);
    else std::cout << "Contour is empty!" << std::endl; 
  }
  Point GetCenter() {
    if(center == Point(-1.0, -1.0)) countCenter();
    return center;
  }
*/
};

template <typename T>
struct Stone3d {
  vector<Point> points; // 3d points of stone
  Stone3d(vector<Point> in_points) : points(in_points) { }
};

/**
*	Find square of figure that has color different from bgcolor
*/
int findSq(Mat markers, int bgcolor) {
  int count = 0;
// TODO: check if input mat is bw
// TODO: the same func with finding exact color
  for( int i = 0; i  < markers.rows; i++) {
    for (int j = 0; j < markers.cols; j++) {
      if (markers.at<int>(i, j) != bgcolor) count++;
    }
  }
  return count;
}

/**
*	Find the center of input contour
*/
Point findContCenter(vector<Point> contour) {

/*  vector<vector<Point>> tmp1 = {};
  tmp1.push_back(contour);
  int k = mldcall(tmp1);
std::cout << "tmpk = " << k << std::endl;*/
  Moments mu;
  mu = moments( contour, false );
  Point2f mc;
  mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
  return mc;
}

/**
*	Find an array of difference between input cintour
*	and circle radius of r with center placed in the
*	center of given contour.
*	Output also saves an information of sines and 
*	cosines of every point of contour (have to be 
*	saved to have a recovery oppotunity).
*/
vector<PolarPoint<double>> compareWithCircle(Mat circleImg, vector<Point> contour, double r) {
/*  vector<vector<Point>> tmp1 = {};
  tmp1.push_back(contour);
  mldcall(tmp1);
*/
  Point center = findContCenter(contour);
  std::cout << "cent = " << center << std::endl; 
  Mat tmp = Mat::zeros(circleImg.size(), CV_32SC1);
  circle(tmp, center, (int)r, CV_RGB(128,128,128)); 
  vector<PolarPoint<double>> difs = {};
  double dif_tmp = 0.0;
  PolarPoint<double> pnt;
  for (int i = 0 ; i < contour.size(); i++) {
    dif_tmp = sqrt((contour[i].x - center.x) * (contour[i].x - center.x) + (contour[i].y - center.y) * (contour[i].y - center.y));
    pnt.rcos = (contour[i].x - center.x) / dif_tmp;
    pnt.rsin = (contour[i].y - center.y) / dif_tmp;
    pnt.r =  dif_tmp - r;
    difs.push_back(pnt);
    difs[i].printCoord(); 
  }
  return difs;
}

/**
*	Recovers contour by data of compared circle and
*	defference from this.
*/

void recoverStone(Mat recStone, vector<PolarPoint<double>> vectorizedStone, Point center, double r) {
  for (int i = 0; i < vectorizedStone.size(); i++) {
     int x = (r + vectorizedStone[i].r) * vectorizedStone[i].rsin + center.y;
     int y = (r + vectorizedStone[i].r) * vectorizedStone[i].rcos + center.x;
     recStone.at<int>(x, y) = 64;
  }

}


/**
*	Extract radiuses of points from PolarPoint array.
*	TODO: make template
*/
vector<double> extractRFromPP(vector<PolarPoint<double>> vecPP) {
  vector<double> retVec = {};
std::cout << "I am extracting! ";
  int nvec = vecPP.size();
  for (int i = 0; i < nvec; i++) {
    retVec.push_back(vecPP[i].r);
  }
std::cout << "Size is " << retVec.size() << std::endl;
  return retVec;
}




vector< P3d<int> > pointCloud(vector<StoneContourPlane<Point>> stonePlanes) {
  int planesSize = stonePlanes.size();
  vector<P3d<int> > out = {};
  for (int i = 0 ; i < planesSize; i++) {
    int nx = stonePlanes[i].contourX.size();
    int ny = stonePlanes[i].contourY.size();
    int nz = stonePlanes[i].contourZ.size();
    for (int ii = 0; ii < nx; i++) {
      int tm = stonePlanes[i].contourX[ii].x;
      out.push_back(P3d<int>(tm, stonePlanes[i].contourX[ii].y, stonePlanes[i].xShift));
    }
    for (int ii = 0; ii < ny; i++) {
      out.push_back(P3d<int>(stonePlanes[i].contourY[ii].x, stonePlanes[i].contourY[ii].y, stonePlanes[i].yShift));
    }
    for (int ii = 0; ii < nz; i++) {
      out.push_back(P3d<int>(stonePlanes[i].contourZ[ii].x, stonePlanes[i].contourZ[ii].y, stonePlanes[i].zShift));
    }
  }
  return out;

}


/*vector< P3d<int> > pointCloud(vector<Mat> imgsX, vector<Mat> imgsY, vector<Mat> imgsZ, 
				int startX = 0, int startY = 0, int startZ = 0,
				int stepX = 1, int stepY = 1, int stepZ = 1, 
				double scaleX = 1.0, double scaleY = 1.0, double scaleZ = 1.0) {
  std::cout << "Step X is  " << stepX << std::endl << "Step Y is " << stepY << std::endl << "Step Z is " << stepZ << std::endl;
  const int nx =  imgsX.size(), ny = imgsY.size(), nz = imgsZ.size();
  std::cout << "nx = " << nx << ", ny = " << ny << ", nz = " << nz << std::endl;
  vector<P3d<int>> outPoints = {};

  int xcoord;
  for (int i = 0; i < nx; i++) {
    xcoord = i * stepX;
    int yMat = imgsX[i].rows, zMat = imgsX[i].cols;
    std::cout << "yMat = " << yMat << ", zMat = " << zMat << std::endl;
    for (int j = 0; j < yMat; j++) {
      for (int k = 0; k < zMat; k++) {
        if (imgsX[i].at<Vec3b>(j, k) == Vec3b(0,0,0)) outPoints.push_back(P3d<int>(startX + xcoord, j, k));
      }
    }
    
  }

  int ycoord;
  for (int i = 0; i < ny; i++) {
    ycoord = i * stepY;
    int yMat = imgsY[i].rows, zMat = imgsY[i].cols;
    for (int j = 0; j < yMat; j++) {
      for (int k = 0; k < zMat; k++) {
        if (imgsY[i].at<Vec3b>(j, k) == Vec3b(0, 0, 0)) outPoints.push_back(P3d<int>(j, startY + ycoord, k));
      }
    }
  }

  int zcoord;
  for (int i = 0; i < nz; i++) {
    zcoord = i * stepZ;
    int yMat = imgsZ[i].rows, zMat = imgsZ[i].cols;
    for (int j = 0; j < yMat; j++) {
      for (int k = 0; k < zMat; k++) {
        if (imgsZ[i].at<Vec3b>(j, k) == Vec3b(0, 0, 0)) outPoints.push_back(P3d<int>(j, k, startZ + zcoord));
      }
    }
  }
  return outPoints;
}
*/
vector<Point> linePoints(int x0, int y0, int x1, int y1)
{
    vector<Point> pointsOfLine;

    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = (dx>dy ? dx : -dy)/2, e2;

    for(;;)
    {
        pointsOfLine.push_back(Point(x0,y0));
        if (x0==x1 && y0==y1) break;
        e2 = err;
        if (e2 >-dx)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y0 += sy;
        }
    }
    return pointsOfLine;
}


vector<Point> makefullcont(vector<Point> in) {
  int s = in.size();
  vector<Point> ret = {};
  ret.push_back(in[0]);
  for (int i = 1; i < s; i++) {
    vector<Point> tmp = {};
    tmp = linePoints(in[i-1].x, in[i-1].y, in[i].x, in[i].y);
    int tmps = tmp.size();
    ret.push_back(in[i]);
    for (int j = 0; j < tmps; j++) {
      ret.push_back(tmp[j]);
      
    }
  }
  return ret;

}




int main(int, char** argv)
{

    Mat src1 = imread("../im1.png");
    Mat src2 = imread("../im2.png");
    Mat src3 = imread("../im3.png");

    vector<Mat> mats1 = {};
    vector<Mat> mats2 = {};
    vector<Mat> mats3 = {};
    mats1.push_back(src1);
    mats2.push_back(src2);
    mats2.push_back(src3);
 //  vector<P3d<int>> pc = pointCloud(mats1, mats2, {}, 1000, 1000, 50);

    std::ofstream ofs ("outtmp.xyz", std::ofstream::out);
   /* int pcsize = pc.size();
std::cout << "pcsize = " << pcsize <<std::endl;
    for (int i = 0; i < pcsize; i++) {
      ofs << pc[i].x << " " << pc[i].y << " " << pc[i].z << " 1 1 1" << std::endl;
    }*/

    vector<Point> out = {};
    out = linePoints(0.0, 0.0, 5.0, 5.0);
    
    for(int i = 0; i < out.size(); i++) {
      ofs << out[i].x << " " << out[i].y << " " << 0 << std::endl;
   }

    ofs.close();
    Mat src = imread(argv[1]);
    // Check if everything was fine
    if (!src.data)
        return -1;
    // Show source image
    imshow("Source Image", src);
    // Change the background from white to black, since that will help later to extract
    // better results during the use of Distance Transform
    for( int x = 0; x < src.rows; x++ ) {
      for( int y = 0; y < src.cols; y++ ) {
          if ( src.at<Vec3b>(x, y) == Vec3b(255,255,255) ) {
            src.at<Vec3b>(x, y)[0] = 0;
            src.at<Vec3b>(x, y)[1] = 0;
            src.at<Vec3b>(x, y)[2] = 0;
          }
	  else 
	  {
            src.at<Vec3b>(x, y)[0] = 255;
            src.at<Vec3b>(x, y)[1] = 255;
            src.at<Vec3b>(x, y)[2] = 255;
	  }  
        }
    }
    // Show output image
    imshow("Black Background Image", src);
    // Create a kernel that we will use for accuting/sharpening our image
    Mat kernel = (Mat_<float>(3,3) <<
            1,  1, 1,
            1, -8, 1,
            1,  1, 1);
    Mat imgLaplacian;
    Mat sharp = src; // copy source image to another temporary one
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    src.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;
    // convert back to 8bits gray scale
    imgResult.convertTo(imgResult, CV_8UC3);

    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    // imshow( "Laplace Filtered Image", imgLaplacian );
    imshow( "New Sharped Image", imgResult );
    src = imgResult; // copy back
    // Create binary image from source image
    Mat bw;
    cvtColor(src, bw, CV_BGR2GRAY);
    threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    imshow("Binary Image", bw);
    // Perform the distance transform algorithm
    Mat dist = bw;
    distanceTransform(bw, dist, CV_DIST_L2, 3);

/*
 vector<P3d<int>> pc = pointCloud(bw, {}, {}, 50, 50, 50);

    std::ofstream ofs ("out.xyz", std::ofstream::out);
    int pcsize = pc.size();
std::cout << "pcsize = " << pcsize <<std::endl;
    for (int i = 0; i < pcsize; i++) {
      ofs << pc[i].x << " " << pc[i].y << " " << pc[i].z << " 1 1 1" << std::endl;
    }
    ofs.close();
*/


    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    normalize(dist, dist, 0, 1., NORM_MINMAX);
    imshow("Distance Transform Image", dist);
    // Threshold to obtain the peaks
    // This will be the markers for the foreground objects

    //Mat dist = src;
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
    vector<vector<Point> > contours;
    vector< Vec4i > hierarchy;
    findContours(dist_8u, contours, hierarchy,  CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Create the marker image for the watershed algorithm
    Mat markers_tmp;
    Mat markers = Mat::zeros(dist.size(), CV_32SC1);
    // Draw the foreground markers
    Mat squares = Mat::zeros(contours.size(), 1, CV_64F); // for squares in points
  vector<vector<Point>> tmp1 = {};
  std::ofstream ofss("out.xyz", std::ofstream::out);

/*
  vector<Point> invec = makefullcont(contours[0]);

  for (int i = 0 ; i < contours[0].size(); i++) {
    ofss << invec[i].x << " " << invec[i].y << " " << 0 << endl;
  }
  ofss.close();
  tmp1.push_back(invec);
  int k = mldcall(tmp1);
std::cout << "tmpk = " << k << std::endl;
*/


std::cout << "contours.size() = " << contours.size() << std::endl;

    for (size_t i = 0; i < contours.size(); i++) {
  	 markers = Mat::zeros(dist.size(), CV_32SC1); 
        drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
        squares.at<double>(i) = (double)findSq(markers, 0);
        std::cout << "squares.at<double>(i)  = " << squares.at<double>(i)  << std::endl;
        double r = sqrt(squares.at<double>(i)  * M_1_PI);
        Point tmp = findContCenter(contours[i]);
        markers_tmp = markers.clone();
        Point center = findContCenter(contours[i]);
        vector<PolarPoint<double>> difs = compareWithCircle(markers_tmp, contours[i], r); 
        recoverStone(markers_tmp, difs, center, r);
 
        


        vector<double> out = extractRFromPP(difs);
        std::cout << "am here!" << std::endl;
        std::cout << "out.size() = " << out.size();
        for(int i = 0; i < out.size(); i++) {
          std::cout << i << " ";
        }
//       recovered = recoverStone(markers_tmp, vectorizedStone,  r);
imshow("Markers-tmp" + to_string(i), markers_tmp*10000);

    

    StoneContourPlane<Point> important;
    important.contourX = contours[0];
    important.contourY = contours[1];
    vector<StoneContourPlane<Point>> inpoints = {};
    inpoints.push_back(important);
    vector< P3d<int> >  outcloud = pointCloud(inpoints);
      std::ofstream ofof("out1.xyz", std::ofstream::out);
    int csize = outcloud.size(); 
    std::cout << "csize = " << csize << std::endl;
    for (int i = 0; i < csize; i++) {
      ofof << outcloud[i].x << " "  << outcloud[i].y << " " << outcloud[i].z << std::endl; 
    }   
    ofof.close();


//       std::cout << "Point = " << tmp << std::endl;
    }
    // Draw the background marker
//    circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
    imshow("Markers", markers*10000);
    // Perform the watershed algorithm
    watershed(src, markers);
    Mat mark = Mat::zeros(markers.size(), CV_8UC1);
    markers.convertTo(mark, CV_8UC1);
    bitwise_not(mark, mark);
    imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
                                  // image looks like at that point
    // Generate random colors
    vector<Vec3b> colors;
    for (size_t i = 0; i < contours.size(); i++)
    {
        int b = theRNG().uniform(0, 255);
        int g = theRNG().uniform(0, 255);
        int r = theRNG().uniform(0, 255);
        colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }
    // Create the result image
    Mat dst = Mat::zeros(markers.size(), CV_8UC3);
    // Fill labeled objects with random colors
std::cout << "markers rows = " << markers.rows << std::endl << "markers cols = " << markers.cols <<std::endl;
//std::cout << colors.size();
//    Mat squares = Mat::zeros(colors.size(), 1, CV_64F); // for squares in points
//  std::cout << "squares = " << squares << std::endl;

    int ttt = 0;
    for (int i = 0; i < markers.rows; i++)
    {
        for (int j = 0; j < markers.cols; j++)
        {
            int index = markers.at<int>(i,j);
            if (index > 0 && index <= static_cast<int>(contours.size())) {
                dst.at<Vec3b>(i,j) = colors[index-1];
//squares.at<double>(index-1) += 1.0;
                ttt +=1;
	    }
            else {
                dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
		//squares.at<int>(0) += 1;	
		ttt +=1;
	    }
        }
    }
    std::cout << "ttt = " << ttt << std::endl; 
    int tm=0;
    imshow("Final Result", dst);

    src = dst;
    for( int i = 0; i< contours.size(); i=hierarchy[i][0] ) // iterate through each contour.
    {
        Rect r= boundingRect(contours[i]);
//std::cout << contours[i] << std::endl;
	std::cout << "hierarchy " << hierarchy[i][0] << " " << hierarchy[i][1] << " " << hierarchy[i][2] << " " << hierarchy[i][3] << std::endl;
        if(hierarchy[i][2]<0) //Check if there is a child contour
          rectangle(src,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,0,255),2,8,0); //Opened contour
        else
          rectangle(src,Point(r.x-10,r.y-10), Point(r.x+r.width+10,r.y+r.height+10), Scalar(0,255,0),2,8,0); //closed contour
    }

    imshow("Final final Result", src);
/* */   waitKey(0);
    return 0;
}
