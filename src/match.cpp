
#include "match.h"

#include <iostream>

#include <opencv2/imgproc.hpp>


std::vector<std::pair<Vector2, Vector2>> MatchFeatures(
   const std::vector<cv::KeyPoint>& keypoints_a,
   const cv::Mat1f& descriptors_a,
   const std::vector<cv::KeyPoint>& keypoints_b,
   const cv::Mat1f& descriptors_b
){
   const double max_feature_distance = 1e4;

   // find initial matches via brute force
   std::cout << "Find initial matches..." << std::endl;
   std::vector<std::pair<Vector2, Vector2>> initial_matches;

   for(int i = 0; i < keypoints_a.size(); ++i)
   {
      int best_index = -1;
      double best_distance = max_feature_distance;

      for(int j = 0; j < keypoints_b.size(); ++j)
      {
         double distance = cv::norm(descriptors_a.row(i) - descriptors_b.row(j), cv::NORM_L2SQR);
         if(distance <= best_distance)
         {
            best_distance = distance;
            best_index = j;
         }
      }

      if(best_index >= 0)
      {
         initial_matches.emplace_back(
            Vector2(keypoints_a[i].pt.x, keypoints_a[i].pt.y),
            Vector2(keypoints_b[best_index].pt.x, keypoints_b[best_index].pt.y)
         );
      }
   }
   std::cout << "Found " << initial_matches.size() << " initial matches" << std::endl;


   // use ransac to find a decent fundamental matrix
   //

   // find matches constrained by fundamental matrix
   std::vector<std::pair<Vector2, Vector2>> final_matches;

   return final_matches;
}


cv::Mat3b RenderMatches(
   const cv::Mat3b& image_a,
   const cv::Mat3b& image_b,
   const std::vector<std::pair<Vector2,Vector2>>& matches
){
   cv::Mat3b render(image_a.size());
   for(int r = 0; r < render.rows; ++r)
      for(int c = 0; c < render.cols; ++c)
      {
         render(r,c) = image_a(r,c)*0.5 + image_b(r,c)*0.5;
      }

   for(const auto & m : matches)
   {
      cv::line(render, cv::Point(m.first(0), m.first(1)), cv::Point(m.second(0), m.second(1)), cv::Vec3b(0,255,255));
   }

   return render;
}

