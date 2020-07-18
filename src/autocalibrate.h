
#ifndef AUTOCALIBRATE_AUTOCALIBRATE_H
#define AUTOCALIBRATE_AUTOCALIBRATE_H

#include "types.h"


/*
   Given a set of feature-to-feature matches, estimate the calibration parameters
   of the image pair. Assumes all parameters have been initialized to reasonable values.
*/
void Autocalibrate(
   const std::vector<std::pair<Vector2,Vector2>> & matches, 
   Matrix33 & rotation,
   Vector3 & translation, 
   Vector2 & center, 
   double &focal
);


#endif
