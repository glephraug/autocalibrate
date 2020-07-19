
#ifndef AUTOCALIBRATE_AUTOCALIBRATE_H
#define AUTOCALIBRATE_AUTOCALIBRATE_H

#include "types.h"


/*
   A model of our camera system.
 */
struct Model
{
   Matrix33 rotation;
   Vector3 translation;
   Vector2 center;
   double focal;
};


// The epipolar error for the given model from the given matches.
void ModelError(
   const std::vector<std::pair<Vector2, Vector2>> & matches,
   const Model & model,
   Eigen::Matrix<double,9,9> & JtJ,
   Eigen::Matrix<double,9,1> & Jtr,
   double & error
);

// Apply a single step to a model.
Model Apply(const Model & model, const Eigen::Matrix<double,9,1> & step);


/*
   Given a set of feature-to-feature matches, estimate the calibration parameters
   of the image pair. Assumes all parameters have been initialized to reasonable values.
*/
void Autocalibrate(
   const std::vector<std::pair<Vector2,Vector2>> & matches, 
   Model & model
);


#endif
