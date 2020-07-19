
#include <iostream>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "types.h"
#include "match.h"
#include "autocalibrate.h"


int main(int argc, char** argv)
{
   if(argc != 3)
   {
      std::cout << "Usage: " << argv[0] << " <image path> <another image path>" << std::endl
                << std::endl
                << "Outputs the principal point, focal length, and relative pose of the two images" << std::endl;

      return 0;
   }

   cv::Mat3b image_a = cv::imread(argv[1]);
   cv::Mat3b image_b = cv::imread(argv[2]);

   // Scale down the images to something more managable.
   // We could probably get a better calibration by using features detected at a higher
   // resolution.
   double scale = std::min(1.0, 480.0/image_a.rows);
   cv::resize(image_a, image_a, cv::Size(), scale, scale);
   cv::resize(image_b, image_b, cv::Size(), scale, scale);

   // Detect features in both images
   cv::Ptr<cv::SIFT> detector = cv::SIFT::create();
   std::vector<cv::KeyPoint> keypoints_a, keypoints_b;
   cv::Mat1f descriptors_a, descriptors_b;
   detector->detectAndCompute(image_a, cv::noArray(), keypoints_a, descriptors_a);
   std::cout << "Found " << keypoints_a.size() << " keypoints in image A" << std::endl;
   detector->detectAndCompute(image_b, cv::noArray(), keypoints_b, descriptors_b);
   std::cout << "Found " << keypoints_b.size() << " keypoints in image B" << std::endl;

   // Find feature matches
   auto matches = MatchFeatures(keypoints_a, descriptors_a, keypoints_b, descriptors_b);
   if(matches.size() < 8)
   {
      std::cout << "Couldn't find enough feature matches, giving up!" << std::endl;
      return 0;
   }

   // Optimize for pose and calibration
   // We could pull good initial guesses out of the fundamental matrix found in
   // the previous step if we wanted to improve results.
   Matrix33 rotation = Matrix33::Identity();
   Vector3 translation = Vector3(0.0, 0.0, 1.0);
   Vector2 center = Vector2(image_a.cols/2, image_a.rows/2);
   double focal = 1e3;

   Autocalibrate(matches, rotation, translation, center, focal);
   
   // Undo the scaling so our data matches the original image sizes
   center /= scale;
   focal /= scale;

   std::cout << "Estimated rotation:" << std::endl << rotation << std::endl;
   std::cout << "Estimated translation: " << translation.transpose() << std::endl;
   std::cout << "Principal point: " << center.transpose() << std::endl;
   std::cout << "Focal length: " << focal << std::endl;

   return 0;
}
