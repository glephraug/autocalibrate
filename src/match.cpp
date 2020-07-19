
#include "match.h"

#include <iostream>
#include <random>

#include <opencv2/imgproc.hpp>

#include "fundamental.h"


/*
   Find the best fundamental matrix via RANSAC.
   We modify the data because it's just quicker and easier. All we do is change 
   the order of some matches.
*/
Matrix33 FundamentalRansac(
   std::vector<std::pair<Vector2, Vector2>> & data
){
   const int iterations = 100;
   const double inlier_distance = 1.0; // pixels

   // We could use mersenne twister or something, but this project isn't really serious.
   std::default_random_engine gen;

   int max_inliers = 0;
   Matrix33 best;
   for(int i = 0; i < iterations; ++i)
   {
      // select 8 data points at random
      std::vector<std::pair<Vector2,Vector2>> subset;
      for(int j = 0; j < 8; ++j){
         std::uniform_int_distribution<int> dist(j, data.size()-1);
         int k = dist(gen);
         std::swap(data[j], data[k]);
         subset.push_back(data[j]);
      }

      Matrix33 F = FundamentalFromMatches(subset);
      
      int inliers = 0;
      for(const auto & m : data)
      {
         if(FundamentalError(F, m.first, m.second) <= inlier_distance) ++inliers;
      }

      if(inliers > max_inliers)
      {
         std::cout << "Found " << inliers << " inliers on iteration " << i << std::endl;
         max_inliers = inliers;
         best = F;
      }
   }

   return best;
}


std::vector<std::pair<Vector2, Vector2>> MatchFeatures(
   const std::vector<cv::KeyPoint> & keypoints_a,
   const cv::Mat1f & descriptors_a,
   const std::vector<cv::KeyPoint> & keypoints_b,
   const cv::Mat1f & descriptors_b
){
   const double max_feature_distance = 1e4; // SIFT feature space
   const double inlier_distance = 1.0; // pixels

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

   if(initial_matches.size() < 8)
   {
      return std::vector<std::pair<Vector2,Vector2>>();
   }

   // We could improve results by doing some kind of angle-based outlier rejection here

   // use ransac to find a decent fundamental matrix
   Matrix33 F = FundamentalRansac(initial_matches);

   // find matches constrained by fundamental matrix
   std::vector<std::pair<Vector2, Vector2>> final_matches;

   for (int i = 0; i < keypoints_a.size(); ++i)
   {
      int best_index = -1;
      double best_distance = max_feature_distance;

      Vector2 va(keypoints_a[i].pt.x, keypoints_a[i].pt.y);

      for (int j = 0; j < keypoints_b.size(); ++j)
      {
         Vector2 vb(keypoints_b[j].pt.x, keypoints_b[j].pt.y);
         if(FundamentalError(F, va, vb) <= inlier_distance)
         {
            double distance = cv::norm(descriptors_a.row(i) - descriptors_b.row(j), cv::NORM_L2SQR);
            if (distance <= best_distance)
            {
               best_distance = distance;
               best_index = j;
            }
         }
      }

      if (best_index >= 0)
      {
         final_matches.emplace_back(va, Vector2(keypoints_b[best_index].pt.x, keypoints_b[best_index].pt.y));
      }
   }
   std::cout << "Found " << final_matches.size() << " final matches" << std::endl;


   return final_matches;
}


cv::Mat3b RenderMatches(
   const cv::Mat3b & image_a,
   const cv::Mat3b & image_b,
   const std::vector<std::pair<Vector2,Vector2>> & matches
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

