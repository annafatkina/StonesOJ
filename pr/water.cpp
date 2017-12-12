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
#include <functional>
#include <memory>


using namespace std;
using namespace cv;


enum Orientation {xOrient, yOrient, zOrient};



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
  P3d() = default;
  P3d(T in_x, T in_y, T in_z) {
    x = in_x; y = in_y; z = in_z;
  }
  T x, y, z;
};

Point findContCenter(vector<Point> contour) {
    Moments mu;
    mu = moments( contour, false );
    Point2f mc;
    mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    return mc;
}

template<typename T>
struct StoneContourPlane {
  vector<T> contour;
  //vector<Point> contourY;
 // vector<Point> contourZ;
    Orientation orient;
  Point center;
  int xShift, yShift, zShift;
  StoneContourPlane() : xShift(0), yShift(0), zShift(0) {
    contour = {};
   // contourY = {};
   // contourZ = {};

  }
    Point getCenter() {
        center = findContCenter(contour);
        return center;
    }
    vector<P3d<double>> get3dContour() {
        int n = contour.size();
        vector<P3d<double>> res = {};
        for (int i = 0; i < n; i++) {
            P3d<double> pointtmp;
            switch (orient) {
                case xOrient: pointtmp = P3d<double>(xShift, contour[i].x + yShift, contour[i].y + zShift);
                    break;
                case yOrient: pointtmp.x = contour[i].x + xShift; pointtmp.y = yShift; pointtmp.z = contour[i].y + zShift;
                    break;
                case zOrient: pointtmp.x = contour[i].x + xShift; pointtmp.y = contour[i].y + yShift; pointtmp.z = zShift;
            }
            res.push_back(pointtmp);
        }
        return res;
    }
    int getStep() {
        switch (orient) {
            case xOrient:
                return xShift;
            case yOrient:
                return yShift;
            case zOrient:
                return zShift;
        }
    }
};

vector<Point> make2d(vector<P3d<double>> contour3d, Orientation orient) {
    int n = contour3d.size();
    vector<Point> res = {};
    for (int i = 0; i < n; i++) {
        switch(orient) {
            case xOrient : res.push_back(Point(contour3d[i].y, contour3d[i].z)); break;
            case yOrient : res.push_back(Point(contour3d[i].x, contour3d[i].z)); break;
            case zOrient : res.push_back(Point(contour3d[i].x, contour3d[i].y)); break;
        }
    }
    return res;
}

vector<Point> makefullcont(vector<Point> in);
int findMaxDim(vector<Point> in);
/**
*	Find an array of difference between input cintour
*	and circle radius of r with center placed in the
*	center of given contour.
*	Output also saves an information of sines and
*	cosines of every point of contour (have to be
*	saved to have a recovery oppotunity).
*/
vector<PolarPoint<double>> compareWithCircle(Mat circleImg, vector<Point> contour, double r, Point center);

/**
*	Recovers contour by data of compared circle and
*	defference from this.
*/

vector<Point> recoverStone(Mat recStone, vector<PolarPoint<double>> vectorizedStone, Point center, double r);



template <typename T>
struct Stone3d {
  vector<vector<P3d<T>>> stoneContours; // 3d points of stone
  int radOfCenter;
    int numOfPoints;
  P3d<T> center;
  Stone3d(vector<vector<P3d<T>>> in_cont, int rad = 0, P3d<T> cent = P3d<T>(0,0,0) ) : stoneContours(in_cont), radOfCenter(rad), center(cent) {
      int k = in_cont.size();
      numOfPoints = 0;
      for(int i = 0; i < k; i++) {
          numOfPoints+=in_cont[i].size();
      }
  }
  void addContourToStone(vector<P3d<T>> in_cont) {
    if (in_cont.size() > 0) {
      stoneContours.push_back(in_cont);
        numOfPoints+=in_cont.size();
      //Point cent = findContCenter(in_cont);
    }
  }
  void toFile(string fileName) {
    std::ofstream ofof(fileName, std::ofstream::out);
    int n = stoneContours.size();
    for(int i = 0; i < n; i++) {
      int nn = stoneContours[i].size();
      for (int ii = 0; ii < nn ; ii++) {
        ofof << stoneContours[i][ii].x << " " << stoneContours[i][ii].y << " " << stoneContours[i][ii].z << std::endl;
      }
    }
    ofof.close();
  }
    void makeDense (int step = 1) {
        int start = center.x - radOfCenter;
        int end = center.x + radOfCenter;
        int upSize = stoneContours.size();
       // Mat tmpMat;
        for (int i = start; i < end; i +=step) {
            for(int j = 0; j < upSize; j++) {
                int downSize = stoneContours[j].size();
                vector<P3d<double >> tmpvec = {};
                for(int k = 0; k < downSize; k++) {
                    if (stoneContours[j][k].x == i) {
                        tmpvec.push_back(stoneContours[j][k]);
                    }
                }
                int tmpvecsize = tmpvec.size();
                if(tmpvecsize > 1){
                    vector<Point> tmp2d = make2d(tmpvec, xOrient);
                    double radOfSmall = 0;
                    for(int ii = 0; ii < tmpvecsize; ii++) {
                        // such strange coors because of xOrient
                        radOfSmall += sqrt((tmp2d[ii].x - center.y)*(tmp2d[ii].x - center.y) + (tmp2d[ii].y - center.z)*(tmp2d[ii].y - center.z));
                    }
                    radOfSmall /= 3.0; // /2

                    int matDim = findMaxDim(tmp2d);
                    cv::Mat tmpMat = Mat::zeros(matDim+1, matDim+1, CV_32SC1);
                    vector<PolarPoint<double>> tmpComparer = compareWithCircle(tmpMat, tmp2d, radOfCenter, Point_<int>(center.y, center.z));
                    int polarSize = tmpComparer.size();
                    for (int kk = 0; kk < polarSize; kk++) {
                        tmpComparer[i].r -= (radOfCenter-radOfSmall);
                    }
                    // todo minim
                    tmp2d = makefullcont(tmp2d);
                    vector<P3d<double >> tmp3d = {};
                    int tmpsize = tmp2d.size();
                    for(int k = 0; k < tmpsize; k++) {
                        tmp3d.push_back(P3d<double >( i, tmp2d[k].x, tmp2d[k].y));
                    }
                    stoneContours.push_back(tmp3d);

                }
            }
        }
    }
};


int findMaxDim(vector<Point> in);
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
P3d<double> findContCenter3dPlane(vector<P3d<double>> contour3d, Orientation orient, int step) {
  vector<Point> contour = make2d(contour3d, orient);
  Moments mu;
  mu = moments( contour, false );
  Point2f mc;
  mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
  P3d<double> res;
  switch(orient) {
    case(xOrient) : res = P3d<double>(step, mc.x, mc.y); break;
    case(yOrient) : res = P3d<double>(mc.x, step, mc.y); break;
    case(zOrient) : res = P3d<double>(mc.x, mc.y, step); break;
  }
  return res;
}





/**
*	Extract radiuses of points from PolarPoint array.
*	TODO: make template
*/
int extractRFromPP(shared_ptr<vector<double>> retVec, vector<PolarPoint<double>> vecPP) {
  //vector<double> retVec = {};
std::cout << "I am extracting! ";
  int nvec = vecPP.size();
  for (int i = 0; i < nvec; i++) {
    retVec->push_back(vecPP[i].r);
    retVec->resize(i+1);
  }
std::cout << "Size is " << retVec->size() << std::endl;
  return retVec->size();
}




/*vector< P3d<int> > pointCloud(vector<StoneContourPlane<Point>> stonePlanes) {
  int planesSize = stonePlanes.size();
  vector<P3d<int> > out = {};
  for (int i = 0 ; i < planesSize; i++) {
    int nx = stonePlanes[i].contourX.size();
    int ny = stonePlanes[i].contourY.size();
    int nz = stonePlanes[i].contourZ.size();
    for (int ii = 0; ii < nx; ii++) {
      int tm = stonePlanes[i].contourX[ii].x;
      out.push_back(P3d<int>(stonePlanes[i].xShift, tm + stonePlanes[i].yShift, stonePlanes[i].contourX[ii].y + stonePlanes[i].zShift));
    }
    for (int ii = 0; ii < ny; ii++) {
      out.push_back(P3d<int>(stonePlanes[i].contourY[ii].x + stonePlanes[i].xShift, stonePlanes[i].yShift, stonePlanes[i].contourY[ii].y + stonePlanes[i].zShift));
    }
    for (int ii = 0; ii < nz; ii++) {
      out.push_back(P3d<int>(stonePlanes[i].contourZ[ii].x + stonePlanes[i].xShift, stonePlanes[i].contourZ[ii].y +stonePlanes[i].yShift, stonePlanes[i].xShift));
    }
  }
  return out;

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


int findMaxDim(vector<Point> in) {
    int inSizr = in.size();
    int max = 0;
    for(int i = 0; i < inSizr; i++) {
        if(in[i].x > max) max = in[i].x;
        if(in[i].y > max) max = in[i].y;

    }
}

vector<PolarPoint<double>> compareWithCircle(Mat circleImg, vector<Point> contour, double r, Point center) {
    //Point center;
    //center = findContCenter(contour);
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
        //difs[i].printCoord();
    }
    return difs;
}

vector<Point> recoverStone(Mat recStone, vector<PolarPoint<double>> vectorizedStone, Point center, double r) {
    vector<Point> points = {};
    for (int i = 0; i < vectorizedStone.size(); i++) {
        int x = (r + vectorizedStone[i].r) * vectorizedStone[i].rsin + center.y;
        int y = (r + vectorizedStone[i].r) * vectorizedStone[i].rcos + center.x;
        recStone.at<int>(x, y) = 64;
        points.push_back(Point_<int>(x, y));
    }
    return points;
}


vector<vector<Point>> extractContFromImg(Mat src) {
  /// Magic
  if (!src.data) {
    std::cout << "err!";
    exit(-1);
  }
        
    // Show source image
    imshow("Source Image", src);
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
    return contours;

}

//enum Orientation {xOrient, yOrient, zOrient};


struct PosedImgs {
  Mat img;
  int beginX, beginY, beginZ;
  Orientation orient;
  
  PosedImgs(Mat in, Orientation orIn) : img(in), orient(orIn), beginX(0), beginY(0), beginZ(0) { }
  PosedImgs(Mat in, Orientation orIn, int x, int y, int z) : img(in), orient(orIn), beginX(x), beginY(y), beginZ(z) { }

  Mat getMat() {
    return img;
  }
};

void addToStones(StoneContourPlane<Point> cont, shared_ptr<vector<Stone3d<double>>>stoneVec, int rad) {
  int nStoneVec = stoneVec->size();
    //cont.center;
    P3d<double> cent;
    switch (cont.orient) {
        case xOrient: cent = P3d<double>(cont.xShift, cont.getCenter().x + cont.yShift, cont.getCenter().y + cont.zShift);
            break;
        case yOrient: cent = P3d<double>(cont.xShift + cont.getCenter().x, cont.yShift, cont.getCenter().y + cont.zShift);
            break;
        case zOrient: cent = P3d<double>(cont.xShift + cont.getCenter().x, cont.yShift + cont.getCenter().y,  cont.zShift);
    }
    bool isExist = false;
    for (int i = 0; i < nStoneVec; i++) {
        double diff = sqrt((cent.x - (*stoneVec)[i].center.x) * (cent.x - (*stoneVec)[i].center.x) +
                           (cent.y - (*stoneVec)[i].center.y) * (cent.y - (*stoneVec)[i].center.y) +
                           (cent.z - (*stoneVec)[i].center.z)*(cent.z - (*stoneVec)[i].center.z) );
        if(diff < (*stoneVec)[i].radOfCenter) {
            (*stoneVec)[i].addContourToStone(cont.get3dContour());
            if ((*stoneVec)[i].radOfCenter < rad) (*stoneVec)[i].radOfCenter = rad; // TODO explain
            isExist = true;
            break;
        }
    }
    if(!isExist) {
        vector<vector<P3d<double>>> newvec = {};
        newvec.push_back(cont.get3dContour());
        Stone3d<double> stone(newvec, rad, findContCenter3dPlane(newvec[0], cont.orient, cont.getStep()));
        stoneVec->push_back(stone);
    }
}


void combineImgs(vector<PosedImgs> imgs) {
  int imgscount = imgs.size();
  //std::ofstream ofof("out.xyz", std::ofstream::out);
 // vector<Stone3d<double>> stone3dVec;
    shared_ptr<vector<Stone3d<double>>> stone3dVecPtr = make_shared<vector<Stone3d<double>>>();
  for (int k = 0; k < imgscount; k++) {
    Mat src(imgs[k].getMat());
    vector<vector<Point>> contours = extractContFromImg(src);
    Mat markers = Mat::zeros(src.size(), CV_32SC1);
    Mat squares = Mat::zeros(contours.size(), 1, CV_64F);
    Mat markers_tmp;
    std::cout << "contours.size() = " << contours.size() << std::endl;
    int conts_size = contours.size();

    for (size_t i = 0; i < conts_size; i++) {
        contours[i] = makefullcont(contours[i]);
/*
 vector<vector<Point>> tmp1 = {};
  tmp1.push_back(contours[i]);
  int k = mldcall(tmp1);
*/
//ofof  <<  << std::endl;

        markers = Mat::zeros(src.size(), CV_32SC1);
        drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
        squares.at<double>(i) = (double)findSq(markers, 0);
        std::cout << "squares.at<double>(i)  = " << squares.at<double>(i)  << std::endl;
        double r = sqrt(squares.at<double>(i)  * M_1_PI);
        Point tmp = findContCenter(contours[i]);
        markers_tmp = markers.clone();
        Point center = findContCenter(contours[i]);
        vector<PolarPoint<double>> difs = compareWithCircle(markers_tmp, contours[i], r, center);
        recoverStone(markers_tmp, difs, center, r);

        shared_ptr<vector<double>> outptr = make_shared<vector<double>>();
        int si = extractRFromPP(outptr, difs);
        vector<double>& outvec =  *outptr;
        int outs = si;
        imshow("Markers-tmp" + to_string(i), markers_tmp*10000);

        StoneContourPlane<Point> important;
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
        stone3dVec[st].makeDense();
    }
  for(int st = 0; st < st3dsize; st++) {
      stone3dVec[st].toFile("ooooooout" + to_string(st) + ".xyz");
  }


  //ofof.close();
}


int main(int, char** argv)
{

    Mat src = imread(argv[1]);
    Mat front1 = imread("../im1.png");
    if (!front1.data) {
    std::cout << "First err!";
    exit(-1);
  }

    Mat front2 = imread("../im2.png");
    Mat front3 = imread("../im3.png");
    //Mat front4 = imread("../im4.png");
    vector<PosedImgs> sources = {};
    //PosedImgs mat1(src, xOrient);
    //sources.push_back(mat1);
    PosedImgs mat0(front1, xOrient, 500, 0, 0);
    //PosedImgs mat1(front4, xOrient);
    PosedImgs mat2(front2, yOrient, -1200, 1700, 0);
    PosedImgs mat3(front3, yOrient, 0, 550, 0);
    sources.push_back(mat0);
    //sources.push_back(mat1);
    sources.push_back(mat2);
    sources.push_back(mat3);
    combineImgs(sources);




//    circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
/*    imshow("Markers", markers*10000);
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
