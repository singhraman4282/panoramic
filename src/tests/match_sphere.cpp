#include <ros/ros.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <panoramic/Panoramic.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spherical_warping");
  ros::NodeHandle nh;

  double focal_length;
  std::string first_path, second_path, output_path;
  std::cout << "Please input the path to the first image: ";
  std::cin >> first_path;
  std::cout << "Please input the path to the second image: ";
  std::cin >> second_path;
  std::cout << "Please input the path to where to save the matched image: ";
  std::cin >> output_path;
  std::cout << "Please input the desired focal length: ";
  std::cin >> focal_length;
  std::cout << std::endl;
  
  std::cout << "Reading images.\n";
  cv::Mat first_sample = cv::imread(first_path.c_str());
  cv::Mat second_sample = cv::imread(second_path.c_str());

  nurc::Panoramic p_server;
  cv::Mat first_mask, second_mask;
  cv::Mat first_warped = p_server.warp_to_hsphere(first_sample, 1000, 1000, focal_length, first_mask);
  cv::Mat second_warped = p_server.warp_to_hsphere(second_sample, 1000, 1000, focal_length, second_mask);
  
  cv::imshow(first_path.c_str(), first_sample);
  cv::imshow(second_path.c_str(), second_sample);
  cv::imshow("First Warped", first_warped);
  cv::imshow("Second Warped", second_warped);

  // Modify for FLANN code
  using namespace cv;

  Mat img_1 = first_warped;
  Mat img_2 = second_warped;

  if( !img_1.data || !img_2.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  detector.detect( img_1, keypoints_1, first_mask );
  detector.detect( img_2, keypoints_2, second_mask );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_1, descriptors_2;

  extractor.compute( img_1, keypoints_1, descriptors_1 );
  extractor.compute( img_2, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
  Mat img_matches;
  drawMatches( img_1, keypoints_1, img_2, keypoints_2,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Show detected matches
  imshow( "Good Matches", img_matches );

  for( int i = 0; i < (int)good_matches.size(); i++ )
  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

  for( int i = 0; i < (int)good_matches.size(); i++ ) { 
    printf( "-- Good Match Points [%d] 1 X-value: %d  1 Y-value: %d -- 2 X-value: %d 2 Y-value: %d \n", 
      i, 
      (int)keypoints_1[(int)good_matches[i].queryIdx].pt.x, 
      (int)keypoints_1[(int)good_matches[i].queryIdx].pt.y, 
      (int)keypoints_2[(int)good_matches[i].trainIdx].pt.x, 
      (int)keypoints_2[(int)good_matches[i].trainIdx].pt.y 
    ); 
  }
  
  cv::imwrite(output_path.c_str(), img_matches);
  
  cv::waitKey(0);
  
  return 0;
}
