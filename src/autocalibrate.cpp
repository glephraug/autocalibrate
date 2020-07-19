
#include "autocalibrate.h"

#include <iostream>


Matrix33 MatrixFromAxisAngle(
   const Vector3 & v
){
   Matrix33 K = Hat(v);

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
   for(const auto & m : matches)
   {
      const Vector2 & a = m.first;
      const Vector2 & b = m.second;
      const Matrix33 & R = model.rotation;
      const Vector3 & t = model.translation;
      const Vector2 & c = model.center;
      double f2 = model.focal*model.focal;

      // I'm sure a lot of this could be optimized much more efficiently

      // normalize coordinates
      Vector3 na((a(0)-model.center(0))/model.focal, (a(1)-model.center(1))/model.focal, 1.0);
      Vector3 nb((b(0)-model.center(0))/model.focal, (b(1)-model.center(1))/model.focal, 1.0);
      Vector3 Rnb = R*nb;

      // The normalized and transformed points should all be coplanar with the cameras.
      // This means the rotated normalized points and baseline should all be coplanar,
      // and their scalar triple product is zero.

      double r = t.dot(na.cross(Rnb));

      Eigen::Matrix<double,1,9> j;

      // rotation
      j.head<3>() = t.cross(na).cross(Rnb).transpose();

      // translation
      j.block<1,3>(0,3) = na.cross(Rnb).transpose();

      // principal point
      Eigen::Matrix<double,3,2> dndc = Eigen::Matrix<double,3,2>::Zero();
      dndc(0,0) = dndc(1,1) = -1.0/model.focal;

      j.block<1,2>(0,6) = -t.cross(Rnb).transpose()*dndc; // drdc from a
      j.block<1,2>(0,6) += t.cross(na).transpose()*R*dndc; // drdc from b

      // focal length
      Vector3 dnadf(-(a(0)-c(0))/f2, -(a(1)-c(1))/f2, 0.0);
      Vector3 dnbdf(-(b(0)-c(0))/f2, -(b(1)-c(1))/f2, 0.0);

      j(8) = -t.cross(Rnb).dot(dnadf); // drdf from a
      j(8) += t.cross(na).dot(R*dnbdf); // drdf from b

      JtJ += j.transpose()*j;
      Jtr += j.transpose()*r;
      error += 0.5*r*r;
   }
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
   std::cout << "initial error: " << error << std::endl;

   for(int i = 0; i < iterations; ++i)
   {
      // solve for lm step
      Matrix99 H = JtJ + (lambda*JtJ.diagonal()).asDiagonal().toDenseMatrix();
      Eigen::JacobiSVD<Matrix99> solver(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Vector9 step = solver.solve(-Jtr);

      Model new_model = Apply(model, step);

      Matrix99 new_JtJ;
      Vector9 new_Jtr;
      double new_error;

      ModelError(matches, new_model, new_JtJ, new_Jtr, new_error);
      std::cout << "iteration " << i << " error: " << new_error << std::endl;

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
