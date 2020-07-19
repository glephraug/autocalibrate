
#ifndef AUTOCALIBRATE_TYPES_H
#define AUTOCALIBRATE_TYPES_H

#include <Eigen/Eigen>

using Vector2 = Eigen::Matrix<double,2,1>;
using Vector3 = Eigen::Matrix<double,3,1>;
using Vector9 = Eigen::Matrix<double,9,1>;

using Matrix33 = Eigen::Matrix<double,3,3>;
using Matrix99 = Eigen::Matrix<double,9,9>;

#endif
