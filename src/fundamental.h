
#ifndef AUTOCALIBRATE_FUNDAMENTAL_H
#define AUTOCALIBRATE_FUNDAMENTAL_H

#include "types.h"


Matrix33 FundamentalFromMatches(const std::vector<std::pair<Vector2,Vector2>> & matches);


#endif
