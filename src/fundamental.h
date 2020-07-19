
#ifndef AUTOCALIBRATE_FUNDAMENTAL_H
#define AUTOCALIBRATE_FUNDAMENTAL_H

#include "types.h"


// Find the matrix F that minimizes a'Fb for all vector pairs (a,b)
Matrix33 FundamentalFromMatches(const std::vector<std::pair<Vector2,Vector2>> & matches);

// Return the maximum distance between a point and the matching epipolar line induced by F
double FundamentalError(const Matrix33 & F, const Vector2 & a, const Vector2 & b);


#endif
