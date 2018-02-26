//
// Created by anna on 26.02.18.
//

#ifndef PR_POLARPOINT_H
#define PR_POLARPOINT_H

#include <iostream>

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
    PolarPoint(T inR, T inRcos, T inRsin)
            : r(inR), rcos(inRcos), rsin(inRsin) {}
    inline void printCoord() {
        std::cout << "r = " << r << ", rcos = " << rcos
                  << ", rsin = " << rsin << std::endl;
    }
};


#endif //PR_POLARPOINT_H
