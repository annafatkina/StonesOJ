//
// Created by anna on 26.02.18.
//

#ifndef PR_POSEDIMGS_H
#define PR_POSEDIMGS_H

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

#include "Orientation.h"

using namespace std;
using namespace cv;


struct PosedImgs {
    Mat img;
    int beginX, beginY, beginZ;
    double scale;
    Orientation orient;
    void imgresize() {
        std::cout << "img.size: " << img.cols << " " << img.rows
                  << std::endl;
        Mat out;
        std::cout << "resize!" << std::endl;
        cv::resize(img, out, Size(), scale, scale);
        img = out.clone();
        beginX *= scale;
        beginY *= scale;
        beginZ *= scale;
        std::cout << "img.size: " << img.cols << " " << img.rows
                  << std::endl;
    }
    PosedImgs(Mat in, Orientation orIn)
            : img(in),
              orient(orIn),
              beginX(0),
              beginY(0),
              beginZ(0),
              scale(1.0) {}
    PosedImgs(Mat in, Orientation orIn, int x, int y, int z)
            : img(in),
              orient(orIn),
              beginX(x),
              beginY(y),
              beginZ(z),
              scale(1.0) {}
    PosedImgs(Mat in, Orientation orIn, double sc)
            : img(in),
              orient(orIn),
              beginX(0),
              beginY(0),
              beginZ(0),
              scale(sc) {
        imgresize();
    }
    PosedImgs(Mat in, Orientation orIn, int x, int y, int z, double sc)
            : img(in),
              orient(orIn),
              beginX(x),
              beginY(y),
              beginZ(z),
              scale(sc) {
        imgresize();
    }
    Mat getMat() { return img; }
};

#endif //PR_POSEDIMGS_H
