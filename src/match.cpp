
#include "match.h"

#include <opencv2/imgproc.hpp>


std::vector<std::pair<Vector2, Vector2>> MatchFeatures(
   const std::vector<cv::KeyPoint>& keypoints_a,
   const cv::Mat1f& descriptors_a,
   const std::vector<cv::KeyPoint>& keypoints_b,
   const cv::Mat1f& descriptors_b
){
   // find initial matches via brute force

   // use ransac to find a decent fundamental matrix

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

