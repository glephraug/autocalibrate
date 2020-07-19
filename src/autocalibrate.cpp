
#include "autocalibrate.h"


Matrix33 MatrixFromAxisAngle(
   const Vector3 & v
){
   Matrix33 K;
   K(0,0) = 0.0;   K(0,1) = -v(2); K(0,2) = v(1);
   K(1,0) = v(2);  K(1,1) = 0.0;   K(1,2) = -v(0);
   K(2,0) = -v(1); K(2,1) = v(0);  K(2,2) = 0.0;

   Matrix33 A = Matrix33::Identity();
   Matrix33 R = A;

   // Sum the Taylor expansion of the exponential map. A few iterations will get us pretty good precision.
   std::uint64_t fac = 1;
   for(int i = 1; i < 5; ++i)
   {
      A *= K;
      fac *= i;
      R += (1.0/fac)*A;
   }

   return R;
}


void ModelError(
   const std::vector<std::pair<Vector2, Vector2>> & matches,
   const Model & model,
   Matrix99 & JtJ,
   Vector9 & Jtr,
   double & error
){
   JtJ.setZero();
   Jtr.setZero();
   error = 0.0;

   // regularization error, keep translation to a unit vector
   // this blows up when translation is zero, but it should never get there.
   {
      double l = model.translation.norm();
      double r = l - 1.0;
      Vector3 j = model.translation * (1.0/l);
      JtJ.block<3,3>(3,3) += j*j.transpose();
      Jtr.block<3,1>(3,0) += j*r;
      error += 0.5*r*r;
   }

   // accumulate errors from matches
}


Model Apply(const Model & model, const Vector9 & step)
{
   Model new_model;

   // After updating the rotation, ensure matrix is orthonormal.
   Matrix33 R = MatrixFromAxisAngle(step.head<3>()) * model.rotation;
   Eigen::JacobiSVD<Matrix33> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
   new_model.rotation = svd.matrixU()*svd.matrixV().transpose();

   new_model.translation = model.translation + step.block<3,1>(3,0);
   new_model.center = model.center + step.block<2,1>(6,0);
   new_model.focal = model.focal + step(8);

   return new_model;
}


void Autocalibrate(
   const std::vector<std::pair<Vector2, Vector2>> & matches,
   Model & model
){
   const int iterations = 100;

   // Levenberg Marquardt optimization
   double lambda = 1.0;
   Matrix99 JtJ;
   Vector9 Jtr;
   double error;

   ModelError(matches, model, JtJ, Jtr, error);

   for(int i = 0; i < iterations; ++i)
   {
      // solve for lm step
      Matrix99 H = JtJ + (lambda*JtJ.diagonal()).asDiagonal().toDenseMatrix();
      Eigen::JacobiSVD<Matrix99> solver(H);
      Vector9 step = solver.solve(-Jtr);

      Model new_model = Apply(model, step);

      Matrix99 new_JtJ;
      Vector9 new_Jtr;
      double new_error;

      ModelError(matches, new_model, new_JtJ, new_Jtr, new_error);

      if(new_error < error)
      {
         lambda /= 2.0;
         JtJ = new_JtJ;
         Jtr = new_Jtr;
         error = new_error;
         model = new_model;
      }
      else
      {
         lambda *= 2.0;
      }
   }
}
