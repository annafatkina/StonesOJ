//
// Created by anna on 26.02.18.
//

#ifndef PR_STONE3D_H
#define PR_STONE3D_H

#include "HelperFunctions.h"
#include "Orientation.h"

template <typename T>
vector<Point> make2d(vector<P3d<T>> contour3d, Orientation orient);

int findMaxDim(vector<Point> in);

vector<PolarPoint<double>> compareWithCircle(Mat circleImg,
                                             vector<Point> contour, double r,
                                             Point center, int& deviation);

double GetContRange(vector<P3d<double>> in, Orientation orient, Point& cent);

vector<Point> recoverStone(vector<PolarPoint<double>> vectorizedStone,
                           Point center, double r,
                           Mat recStone = Mat::zeros(0, 0, CV_32SC1));


template <typename T>
struct Stone3d {
    vector<vector<P3d<T>>> stoneContours;  // 3d points of stone
    int radOfCenter;
    int numOfPoints;
    P3d<T> center;
    Stone3d(vector<vector<P3d<T>>> in_cont, int rad = 0,
            P3d<T> cent = P3d<T>(0, 0, 0))
    : stoneContours(in_cont), radOfCenter(rad), center(cent) {
        int k = in_cont.size();
        numOfPoints = 0;
        for (int i = 0; i < k; i++) {
            numOfPoints += in_cont[i].size();
        }
    }
    void addContourToStone(vector<P3d<T>> in_cont) {
        if (in_cont.size() > 0) {
            stoneContours.push_back(in_cont);
            numOfPoints += in_cont.size();
            // Point cent = findContCenter(in_cont);
        }
    }
    int getContoursNum() { return static_cast<int>(stoneContours.size()); }
    void toFile(string fileName) {
        std::ofstream ofof(fileName, std::ofstream::out);
        int n;
        n = static_cast<int>(stoneContours.size());
        for (int i = 0; i < n; i++) {
            int nn = stoneContours[i].size();
            for (int ii = 0; ii < nn; ii++) {
                ofof << stoneContours[i][ii].x << " "
                     << stoneContours[i][ii].y << " "
                     << stoneContours[i][ii].z << std::endl;
            }
        }
        ofof.close();
    }
    vector<P3d<double>> crossSection(int in, Orientation CSorient,
                                     int step = 1) {
        int upSize;
        upSize = static_cast<int>(stoneContours.size());
        vector<P3d<double>> out = {};
        for (int i = 0; i < upSize; i++) {
            int downSize = stoneContours[i].size();
            for (int j = 0; j < downSize; j++) {
                switch (CSorient) {
                    case xOrient:
                        if (abs(stoneContours[i][j].x -
                                in) < step)
                            out.push_back(
                                    stoneContours[i]
                                    [j]);
                        break;
                    case yOrient:
                        if (abs(stoneContours[i][j].y ==
                                in) < step)
                            out.push_back(
                                    stoneContours[i]
                                    [j]);
                        break;
                    case zOrient:
                        if (abs(stoneContours[i][j].z -
                                in) < step)
                            out.push_back(
                                    stoneContours[i]
                                    [j]);
                        break;
                }
            }
        }
        return out;
    }
    void makeDense(int step = 1) {
        int start = 0;
        int end = 0;
        int numOfConts = static_cast<int>(stoneContours.size());
        std::cout << "MakeDense  Center Point = (" << center.x << ", "
                  << center.y << ", " << center.z << ")" << std::endl;
        Orientation denseOrientation;
        if (numOfConts != 0) {
            int iterCont = -1;
            int contPointNum = 0;
            for (int i = 0; i < numOfConts; i++) {
                contPointNum = stoneContours[i].size();
                if (contPointNum > 1) {
                    iterCont = i;
                    break;
                }
            }
            if (iterCont == -1) {
                std::cerr
                        << "Can't make dense! More points needed!"
                        << std::endl;
                return;
            }
            bool majorOrientChecker;
            int collapsedCoord = 0;
            majorOrientChecker = stoneContours[iterCont][0].x ==
                                 stoneContours[iterCont][1].x &&
                                 stoneContours[iterCont][1].x ==
                                 stoneContours[iterCont][2].x;
            if (majorOrientChecker) {
                denseOrientation = xOrient;
                collapsedCoord = stoneContours[iterCont][0].x;
                start = center.x - radOfCenter;
                end = center.x + radOfCenter;
            } else {
                majorOrientChecker =
                        stoneContours[iterCont][0].y ==
                        stoneContours[iterCont][1].y &&
                        stoneContours[iterCont][1].y ==
                        stoneContours[iterCont][2].y;

                if (majorOrientChecker) {
                    denseOrientation = yOrient;
                    collapsedCoord =
                            stoneContours[iterCont][0].y;
                    start = center.y - radOfCenter;
                    end = center.y + radOfCenter;
                } else {
                    majorOrientChecker =
                            stoneContours[iterCont][0].z ==
                            stoneContours[iterCont][1].z &&
                            stoneContours[iterCont][1].z ==
                            stoneContours[iterCont][2].z;
                    if (majorOrientChecker) {
                        denseOrientation = zOrient;
                        collapsedCoord =
                                stoneContours[iterCont]
                                [0].z;
                        start = center.z - radOfCenter;
                        end = center.z + radOfCenter;
                    }
                }
            }
            Point center2d;
            switch (denseOrientation) {
                case xOrient:
                    center2d = Point(center.y, center.z);
                    break;
                case yOrient:
                    center2d = Point(center.x, center.z);
                    break;
                case zOrient:
                    center2d = Point(center.x, center.y);
                    break;
            }
            std::cout << "MakeDense  Center Point 2d = ("
                      << center2d.x << ", " << center2d.y << ")"
                      << std::endl;
            vector<P3d<double>> templateContour =
                    stoneContours[iterCont];
            vector<Point> tCont2d =
                    make2d(templateContour, denseOrientation);
            int templateSize = templateContour.size();
            double temolateRange = 0;
            vector<int> rangeVec = {};
            for (int i = 0; i < templateSize; i++) {
                if (abs(tCont2d[i].x - center2d.x) < step) {
                    rangeVec.push_back(tCont2d[i].y);
                }
            }
            templateSize = rangeVec.size();
            for (int i = 0; i < templateSize; i++) {
                for (int j = 0; j < templateSize; j++) {
                    if (abs(rangeVec[i] - rangeVec[j]) >
                        temolateRange)
                        temolateRange = abs(
                                rangeVec[i] - rangeVec[j]);
                }
            }
            temolateRange /= 2;
            int dim = findMaxDim(tCont2d);
            Mat tmpMat = Mat::zeros(dim + 1, dim + 1, CV_32SC1);
            int deviation;

            vector<PolarPoint<double>> polar2d = compareWithCircle(
                    tmpMat, tCont2d, radOfCenter, center2d, deviation);
            int polar2dsize = polar2d.size();
            std::cout << "collaps " << collapsedCoord << std::endl;
            for (int thirdCoord = start; thirdCoord < end;
                 thirdCoord++) {
                if (thirdCoord % step != 0) continue;
                if (abs(thirdCoord - collapsedCoord) < step)
                    continue;
                vector<P3d<double>> curCS = crossSection(
                        thirdCoord, denseOrientation, step);
                double maxRange = GetContRange(
                        curCS, denseOrientation, center2d);
                if (maxRange <= step) continue;
                vector<PolarPoint<double>> curPolar2d = polar2d;
                std::cout
                        << thirdCoord << " recovs = "
                        << radOfCenter /
                           (temolateRange / (maxRange / 2))
                        << " maxrang = " << maxRange << std::endl;
                vector<Point> tmpv = recoverStone(
                        curPolar2d, center2d,
                        radOfCenter /
                        (temolateRange / (maxRange / 2)));
                vector<P3d<double>> out = {};

                std::cout
                        << "range = "
                        << (double)(temolateRange / (maxRange / 2))
                        << std::endl;
                int tmpvSize = tmpv.size();
                for (int ii = 0; ii < tmpvSize; ii++) {
                    switch (denseOrientation) {
                        case xOrient:
                            out.push_back(
                                    P3d<double>(
                                            thirdCoord,
                                            tmpv[ii].y,
                                            tmpv[ii].x));
                            break;
                        case yOrient:
                            out.push_back(
                                    P3d<double>(
                                            tmpv[ii].y,
                                            thirdCoord,
                                            tmpv[ii].x));
                            break;
                        case zOrient:
                            out.push_back(
                                    P3d<double>(
                                            tmpv[ii].y,
                                            tmpv[ii].x,
                                            thirdCoord));
                            break;
                    }
                }
                stoneContours.push_back(out);
            }
            stoneContours.erase(stoneContours.begin(),
                                stoneContours.begin() + numOfConts);
        }
    }
};





#endif //PR_STONE3D_H
