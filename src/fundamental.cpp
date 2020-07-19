
#include "fundamental.h"


Matrix33 FundamentalFromMatches(const std::vector<std::pair<Vector2, Vector2>> & matches)
{
   Eigen::MatrixXd A(matches.size(), 9);

   for(int i = 0; i < matches.size(); ++i)
   {
      const Vector2 & a = matches[i].first;
      const Vector2 & b = matches[i].second;
      A(i,0) = a(0)*b(0);
      A(i,1) = a(0)*b(1);
      A(i,2) = a(0);
      A(i,3) = a(1)*b(0);
      A(i,4) = a(1)*b(1);
      A(i,5) = a(1);
      A(i,6) = b(0);
      A(i,7) = b(1);
      A(i,8) = 1.0;
   }

   // Use svd to find F with least squared error
   Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

   // Reshape last eigenvector into a matrix
   Eigen::VectorXd ev = svd.matrixV().col(8);
   Matrix33 F;
   F.row(0) = ev.head<3>().transpose();
   F.row(1) = ev.block<3,1>(3,0).transpose();
   F.row(2) = ev.tail<3>().transpose();

   // Force internal fundamental matrix constraint
   Eigen::JacobiSVD<Eigen::Matrix3d> svd3(F);
   Vector3 s = svd3.singularValues();
   s(2) = 0.0;
   F = svd3.matrixU() * s.asDiagonal() * svd3.matrixV().transpose();

   return F;
}


double FundamentalError(const Matrix33& F, const Vector2& a, const Vector2& b)
{
   Vector3 ha(a(0), a(1), 1.0);
   Vector3 hb(b(0), b(1), 1.0);
   Vector3 la = F.transpose()*ha;
   Vector3 lb = F*b;

   // normalize lines
   la /= la.head<2>().norm();
   lb /= lb.head<2>().norm();

   // Return whichever distance between point and epipolar line is higher
   return std::max(ha.dot(lb), hb.dot(la));
}
