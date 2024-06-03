
#ifndef CYBER_GUIDE_MATHTYPES_H
#define CYBER_GUIDE_MATHTYPES_H

#include <eigen3/Eigen/Dense>

/************************/
/******** Vector ********/
/************************/
// 2x1 Vector
using Vec2d = typename Eigen::Matrix<double, 2, 1>;

// 3x1 Vector
using Vec3d = typename Eigen::Matrix<double, 3, 1>;

// 4x1 Vector
using Vec4d = typename Eigen::Matrix<double, 4, 1>;

// 6x1 Vector
using Vec6d = typename Eigen::Matrix<double, 6, 1>;

// Quaternion
using Quatd = typename Eigen::Matrix<double, 4, 1>;

// 4x1 Integer Vector
using VecInt4d = typename Eigen::Matrix<int, 4, 1>;

// 12x1 Vector
using Vec12d = typename Eigen::Matrix<double, 12, 1>;

// 18x1 Vector
using Vec18d = typename Eigen::Matrix<double, 18, 1>;

// Dynamic Length Vector
using VecXd = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/
// Rotation Matrix
using RotMatd = typename Eigen::Matrix<double, 3, 3>;

// Homogenous Matrix
using HomoMatd = typename Eigen::Matrix<double, 4, 4>;

// 2x2 Matrix
using Mat2d = typename Eigen::Matrix<double, 2, 2>;

// 3x3 Matrix
using Mat3d = typename Eigen::Matrix<double, 3, 3>;

// 3x3 Identity Matrix
#define I3 Eigen::MatrixXd::Identity(3, 3)

// 3x4 Matrix, each column is a 3x1 vector
using Vec34d = typename Eigen::Matrix<double, 3, 4>;

// 6x6 Matrix
using Mat6d = typename Eigen::Matrix<double, 6, 6>;

// 12x12 Matrix
using Mat12d = typename Eigen::Matrix<double, 12, 12>;

// 12x12 Identity Matrix
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 Identity Matrix
#define I18 Eigen::MatrixXd::Identity(18, 18)

// Dynamic Size Matrix
using MatXd = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/************************/
/****** Functions *******/
/************************/
inline Vec34d vec12ToVec34(Vec12d vec12){
    Vec34d vec34;
    for(int i(0); i < 4; ++i){
        vec34.col(i) = vec12.segment(3*i, 3);
    }
    return vec34;
}

inline Vec12d vec34ToVec12(Vec34d vec34){
    Vec12d vec12;
    for(int i(0); i < 4; ++i){
        vec12.segment(3*i, 3) = vec34.col(i);
    }
    return vec12;
}


#endif //CYBER_GUIDE_MATHTYPES_H
