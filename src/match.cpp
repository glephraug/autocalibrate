
#include "match.h"


std::vector<Vector2, Vector2> MatchFeatures(
   const std::vector<cv::KeyPoint>& keypoints_a,
   const cv::Mat1f& descriptors_a,
   const std::vector<cv::KeyPoint>& keypoints_b,
   const cv::Mat1f& descriptors_b
);


cv::Mat3b RenderMatches(
   const cv::Mat3b& image_a,
   const cv::Mat3b& image_b,
   const std::vector<Vector2, Vector2>& matches
);

