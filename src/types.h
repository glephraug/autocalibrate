
#ifndef AUTOCALIBRATE_TYPES_H
#define AUTOCALIBRATE_TYPES_H

#include <Eigen/Eigen>

using Vector2 = Eigen::Matrix<double,2,1>;
using Vector3 = Eigen::Matrix<double,3,1>;
using Vector9 = Eigen::Matrix<double,9,1>;

using Matrix33 = Eigen::Matrix<double,3,3>;
using Matrix99 = Eigen::Matrix<double,9,9>;


inline Matrix33 Hat(const Vector3 & v)
{
   Matrix33 h;
   h(0,0) = 0.0;   h(0,1) = -v(2); h(0,2) = v(1);
   h(1,0) = v(2);  h(1,1) = 0.0;   h(1,2) = -v(0);
   h(2,0) = -v(1); h(2,1) = v(0);  h(2,2) = 0.0;
   return h;
}


#endif
