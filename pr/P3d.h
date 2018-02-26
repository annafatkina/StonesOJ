//
// Created by anna on 26.02.18.
//

#ifndef PR_P3D_H
#define PR_P3D_H

template <typename T>
struct P3d {
    typedef T value_type;
    P3d() = default;
    P3d(T in_x, T in_y, T in_z) {
        x = in_x;
        y = in_y;
        z = in_z;
    }
    T x, y, z;
};

#endif //PR_P3D_H
