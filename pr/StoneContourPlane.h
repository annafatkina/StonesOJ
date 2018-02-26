//
// Created by anna on 26.02.18.
//

#ifndef PR_STONECONTOURPLANE_H
#define PR_STONECONTOURPLANE_H

#include "HelperFunctions.h"


//Point findContCenter(vector<Point> contour);





// template <typename T>
struct StoneContourPlane {
    vector<Point> contour;
    Orientation orient;
    Point center;
    int xShift, yShift, zShift;

    StoneContourPlane() : xShift(0), yShift(0), zShift(0) { contour = {}; }

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
                case xOrient:
                    pointtmp = P3d<double>(
                            xShift, contour[i].x + yShift,
                            contour[i].y + zShift);
                    break;
                case yOrient:
                    pointtmp.x = contour[i].x + xShift;
                    pointtmp.y = yShift;
                    pointtmp.z = contour[i].y + zShift;
                    break;
                case zOrient:
                    pointtmp.x = contour[i].x + xShift;
                    pointtmp.y = contour[i].y + yShift;
                    pointtmp.z = zShift;
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

void addToStones(StoneContourPlane cont, shared_ptr<vector<Stone3d<double>>> stoneVec, int rad);


#endif //PR_STONECONTOURPLANE_H
