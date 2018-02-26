//
// Created by anna on 26.02.18.
//

#include "HelperFunctions.h"
#include "StoneContourPlane.h"
#include "Stone3d.h"



template<typename T>
T signum(T in) {
    if (in < 0) return -1;
    if (in > 0) return 1;
    if (in == 0) return 0;
}


Point findContCenter(vector<Point> contour) {
    Moments mu;
    mu = moments(contour, false);
    Point2f mc;
    mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    return mc;
}


template <typename T>
vector<Point> make2d(vector<P3d<T>> contour3d, Orientation orient) {
    int n = contour3d.size();
    vector<Point> res = {};
    for (int i = 0; i < n; i++) {
        switch (orient) {
            case xOrient:
                res.push_back(
                        Point(contour3d[i].y, contour3d[i].z));
                break;
            case yOrient:
                res.push_back(
                        Point(contour3d[i].x, contour3d[i].z));
                break;
            case zOrient:
                res.push_back(
                        Point(contour3d[i].x, contour3d[i].y));
                break;
        }
    }
    return res;
}

vector<Point> lineP(int x0, int y0, int x1, int y1, int step) {
    vector<Point> pointsOfLine;

    int dx = abs(x1 - x0), sx = step * (x0 < x1 ? 1 : -1);
    int dy = abs(y1 - y0), sy = step * (y0 < y1 ? 1 : -1);

    int err = (dx > dy ? dx : -dy) / 2, e2;
    int counter = 0;
    for (;;) {
        counter++;
        if (abs(x0 - x1) < step && abs(y0 - y1) < step) break;
        if (counter % step != 0) {
            continue;
        }
        pointsOfLine.push_back(Point(x0, y0));
        e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
    return pointsOfLine;
}


vector<Point> makefullcont(vector<Point> in, int step) {
    int s = in.size();
    vector<Point> ret = {};
    if (s != 0) ret.push_back(in[0]);
    int tmps = 0;
    for (int i = 1; i < s; i++) {
        vector<Point> tmp = {};
        tmp = lineP(in[i - 1].x, in[i - 1].y, in[i].x, in[i].y, step);
        tmps = tmp.size();
        ret.push_back(in[i]);
        for (int j = 0; j < tmps; j++) {
            ret.push_back(tmp[j]);
        }
    }

    vector<Point> tmp =
            lineP(in[s - 1].x, in[s - 1].y, in[0].x, in[0].y, step);
    tmps = tmp.size();
    ret.push_back(in[0]);
    for (int j = 0; j < tmps; j++) {
        ret.push_back(tmp[j]);
    }
    return ret;
}




double GetContRange(vector<P3d<double>> in, Orientation orient, Point &cent) {
    vector<Point> in2d = make2d(in, orient);
    double max = 0;
    int inSize = in2d.size();
    if (inSize <= 1) {
        return 0;
    }
    int counter = 0;
    for (int k = 0; k < inSize; k++) {
        for (int i = 0; i < inSize; i++) {
            if (i == k) continue;
            double tmp = sqrt(
                    (in2d[i].x - in2d[k].x) * (in2d[i].x - in2d[k].x) +
                    (in2d[i].y - in2d[k].y) * (in2d[i].y - in2d[k].y));
            if (tmp > max) {
                max = tmp;
                counter++;
                cent.y = (in2d[i].y + in2d[k].y) / 2;
            }
        }
    }
    return max;
}



P3d<double> findContCenter3dPlane(vector<P3d<double>> contour3d, Orientation orient, int step) {
    vector<Point> contour = make2d(contour3d, orient);
    Moments mu;
    mu = moments(contour, false);
    Point2f mc;
    mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    P3d<double> res;
    switch (orient) {
        case (xOrient):
            res = P3d<double>(step, mc.x, mc.y);
            break;
        case (yOrient):
            res = P3d<double>(mc.x, step, mc.y);
            break;
        case (zOrient):
            res = P3d<double>(mc.x, mc.y, step);
            break;
    }
    return res;
}




int findMaxDim(vector<Point> in) {
    int inSizr = in.size();
    int max = 0;
    for (int i = 0; i < inSizr; i++) {
        if (in[i].x > max) max = in[i].x;
        if (in[i].y > max) max = in[i].y;
    }
    return max;
}

vector<PolarPoint<double>> compareWithCircle(Mat circleImg,
                                             vector<Point> contour, double r,
                                             Point center, int& deviation) {
    Mat tmp = Mat::zeros(circleImg.size(), CV_32SC1);
    circle(tmp, center, (int)r, 128);
    vector<PolarPoint<double>> difs = {};
    double dif_tmp = 0.0;
    int maxdev = 0;
    PolarPoint<double> pnt;
    int contsize = contour.size();
    for (int i = 0; i < contsize; i++) {
        dif_tmp =
                sqrt((contour[i].x - center.x) * (contour[i].x - center.x) +
                     (contour[i].y - center.y) * (contour[i].y - center.y));
        pnt.rcos = (contour[i].x - center.x) / dif_tmp;
        pnt.rsin = (contour[i].y - center.y) / dif_tmp;
        pnt.r = dif_tmp - r;
        maxdev += pnt.r;
        difs.push_back(pnt);
    }
    deviation = maxdev / contsize;
    return difs;
}

vector<Point> recoverStone(vector<PolarPoint<double>> vectorizedStone,
                           Point center, double r, Mat recStone) {
    vector<Point> points = {};
    int vStoneSize = vectorizedStone.size();
    for (int i = 0; i < vStoneSize; i++) {
        int x, y;
        int multiplier = r + vectorizedStone[i].r;
        if (multiplier < 0) multiplier = 0;
        x = static_cast<int>(multiplier * vectorizedStone[i].rsin +
                             center.y);
        y = static_cast<int>(multiplier * vectorizedStone[i].rcos +
                             center.x);

        if (recStone.cols != 0) {
            recStone.at<int>(x, y) = 64;
        }
        points.push_back(Point(x, y));
    }
    return points;
}

int findSq(Mat markers, int bgcolor) {
    int count = 0;
    // TODO: check if input mat is bw
    // TODO: the same func with finding exact color
    for (int i = 0; i < markers.rows; i++) {
        for (int j = 0; j < markers.cols; j++) {
            if (markers.at<int>(i, j) != bgcolor) count++;
        }
    }
    return count;
}

void addToStones(StoneContourPlane cont, shared_ptr<vector<Stone3d<double>>> stoneVec, int rad) {
    int nStoneVec = stoneVec->size();
// cont.center;
    P3d<double> cent;
    switch (cont.orient) {
        case xOrient:
            cent = P3d<double>(cont.xShift,
                               cont.getCenter().x + cont.yShift,
                               cont.getCenter().y + cont.zShift);
            break;
        case yOrient:
            cent = P3d<double>(cont.xShift + cont.getCenter().x,
                               cont.yShift,
                               cont.getCenter().y + cont.zShift);
            break;
        case zOrient:
            cent = P3d<double>(cont.xShift + cont.getCenter().x,
                               cont.yShift + cont.getCenter().y,
                               cont.zShift);
    }
    bool isExist = false;
    for (int i = 0; i < nStoneVec; i++) {
        double diff = sqrt((cent.x - (*stoneVec)[i].center.x) *
                           (cent.x - (*stoneVec)[i].center.x) +
                           (cent.y - (*stoneVec)[i].center.y) *
                           (cent.y - (*stoneVec)[i].center.y) +
                           (cent.z - (*stoneVec)[i].center.z) *
                           (cent.z - (*stoneVec)[i].center.z));
        if (diff < (*stoneVec)[i].radOfCenter) {
            (*stoneVec)[i].addContourToStone(cont.get3dContour());
            if ((*stoneVec)[i].radOfCenter < rad)
                (*stoneVec)[i].radOfCenter =
                        rad;  // TODO explain
            isExist = true;
            break;
        }
    }
    if (!isExist) {
        vector<vector<P3d<double>>> newvec = {};
        newvec.push_back(cont.get3dContour());
        Stone3d<double> stone(
                newvec, rad, findContCenter3dPlane(newvec[0], cont.orient,
                                                   cont.getStep()));

        stoneVec->push_back(stone);
    }
}


vector<vector<Point>> extractContFromImg(Mat src) {
    /// Magic
    if (!src.data) {
        std::cout << "err!";
        exit(-1);
    }
    std::cout << "src type = " << src.type() << " ";
    // Show source image
    for (int x = 0; x < src.rows; x++) {
        for (int y = 0; y < src.cols; y++) {
            if (src.at<Vec3b>(x, y) == Vec3b(255, 255, 255)) {
                src.at<Vec3b>(x, y)[0] = 0;
                src.at<Vec3b>(x, y)[1] = 0;
                src.at<Vec3b>(x, y)[2] = 0;
            } else {
                src.at<Vec3b>(x, y)[0] = 255;
                src.at<Vec3b>(x, y)[1] = 255;
                src.at<Vec3b>(x, y)[2] = 255;
            }
        }
    }
    // Create a kernel that we will use for accuting/sharpening our image
    Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
    Mat imgLaplacian;
    Mat sharp = src;  // copy source image to another temporary one
    filter2D(sharp, imgLaplacian, CV_32F, kernel);
    src.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;
    imgResult.convertTo(imgResult, CV_8UC3);

    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);
    src = imgResult;  // copy back
    // Create binary image from source image
    Mat bw;
    cvtColor(src, bw, CV_BGR2GRAY);
    threshold(bw, bw, 40, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // Perform the distance transform algorithm
    Mat dist = bw;
    distanceTransform(bw, dist, CV_DIST_L2, 3);

    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    normalize(dist, dist, 0, 1., NORM_MINMAX);
    // Threshold to obtain the peaks
    // This will be the markers for the foreground objects

    // Mat dist = src;
    threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
    // Dilate a bit the dist image
    Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
    dilate(dist, dist, kernel1);
    // Create the CV_8U version of the distance image
    // It is needed for findContours()
    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find total markers
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(dist_8u, contours, hierarchy, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE);
    return contours;
}


int extractRFromPP(shared_ptr<vector<double>> retVec,
                   vector<PolarPoint<double>> vecPP) {
    int nvec = vecPP.size();
    for (int i = 0; i < nvec; i++) {
        retVec->push_back(vecPP[i].r);
    }
    return retVec->size();
}