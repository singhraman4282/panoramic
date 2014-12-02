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

using namespace nurc;

Panoramic::Panoramic() :
  nh_()
{
  image_stitcher_ = nh_.advertiseService<SphericalStitchRequest, SphericalStitchResponse>("/stitch", boost::bind(&Panoramic::stitch, this, _1, _2));
}

Panoramic::~Panoramic()
{

}

bool Panoramic::stitch(SphericalStitchRequest& req, SphericalStitchResponse& res)
{
  double focal_length = 300;      // Approximate -- needs better defining
  int theta_res, phi_res;
  if(req.phi_res == SphericalStitchRequest::MAX_RES)
    phi_res = (int)round(double( 2. * M_PI ) / atan( 1. /focal_length ) );
  if(req.theta_res == SphericalStitchRequest::MAX_RES)
    theta_res = (int)round(double( 2. * M_PI ) / atan( 1. / focal_length ) );

  cv::Mat sphere( phi_res, theta_res, CV_8UC3 );
  std::vector< std::pair<cv::Mat, cv::Mat> > image_queue;
  for( int i = 0; i < req.queue.size(); i++ ) {
    cv_bridge::CvImagePtr input_image = cv_bridge::toCvCopy( req.queue[i], sensor_msgs::image_encodings::BGR8 );
    cv::Mat mask;
    image_queue.push_back( std::pair<cv::Mat, cv::Mat>(map_to_sphere( input_image->image, phi_res, theta_res, focal_length, mask ), mask) );
  }

  generate_spherical_stitch(sphere, image_queue, phi_res, theta_res);
  
  return true;
}

cv::Mat Panoramic::map_to_sphere(cv::Mat& input, int phi_res, int theta_res, int focal_length, cv::Mat& mask)
{
  // Half-warping

  double d_theta = atan(1.) / double(focal_length);
  double d_phi = atan(1. / double(focal_length) );
  double R_theta = d_theta * theta_res / M_PI;
  double R_phi = d_phi * phi_res / M_PI;
  int R = ceil(std::max( R_theta, R_phi ));
  
  cv::Mat scaled_input;
  cv::resize( input, scaled_input, cv::Size( ceil(R)*input.rows, ceil(R)*input.cols ) );

  /*std::cout << d_theta << std::endl;
  std::cout << d_phi << std::endl;
  std::cout << R_theta << std::endl;
  std::cout << R_phi << std::endl;
  std::cout << R << std::endl;
  std::cout << ceil(R) << std::endl;*/
  
  cv::Mat warp( phi_res, theta_res, CV_8UC3 );
  mask = cv::Mat::zeros( phi_res, theta_res, CV_8UC1 );
  for(int y = 0; y < scaled_input.rows; y++) {
    for(int x = 0; x < scaled_input.cols; x++) {
      double y_disp = double(y-scaled_input.rows/2)/double(R);
      double x_disp = double(x-scaled_input.cols/2)/double(R);
      int phi = ( atan( y_disp / pow( pow( x_disp, 2 ) + pow( double(focal_length), 2 ) , 0.5 ) )
          + M_PI/2. ) * double( phi_res ) / M_PI;
      int theta = atan( x_disp / double(focal_length) ) * double( theta_res ) / M_PI;
      if(theta >= theta_res/2)
        theta = theta-theta_res/2;
      else
        theta = theta+theta_res/2;
      warp.at<cv::Vec3b>(phi, theta) = scaled_input.at<cv::Vec3b>(y, x);
      mask.at<unsigned char>(phi, theta) = 1;
    }
  } 

  return warp;
}

void Panoramic::generate_spherical_stitch(cv::Mat& sphere, std::vector< std::pair<cv::Mat, cv::Mat> >& warped_inputs, int phi_res, int theta_res)
{
  // Algorithm
  // Select the first image to be the center
  // Incrementally add elements to the spherial warping
  //   Each time an element is added run an image alignment over it
  
  for(int i = 0; i < warped_inputs.size()-1; i++){
    cv::Mat first_image = warped_inputs[i].first;
    cv::Mat second_image = warped_inputs[i+1].first;
    cv::Mat first_mask = warped_inputs[i].second;
    cv::Mat second_mask = warped_inputs[i+1].second;
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    cv::SurfFeatureDetector detector( minHessian );
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    detector.detect( first_image, keypoints_1, first_mask);
    detector.detect( second_image, keypoints_2, second_mask);

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_1, descriptors_2;
    extractor.compute( first_image, keypoints_1, descriptors_1 );
    extractor.compute( second_image, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches
    std::vector< cv::DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
      if( matches[i].distance <= std::max(2*min_dist, 0.02) )
        good_matches.push_back( matches[i]); 
    }
    cv::Mat img_matches;
    drawMatches( first_image, keypoints_1, second_image, keypoints_2, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //-- Show detected matches
    imshow( "Good Matches", img_matches );
 /*   for( int i = 0; i < (int)good_matches.size(); i++ ) {
      printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx );
    }

    for( int i = 0; i < (int)good_matches.size(); i++ ) { 
      printf( "-- Good Match Points [%d] 1 X-value: %d  1 Y-value: %d -- 2 X-value: %d 2 Y-value: %d \n", i, 
      (int)keypoints_1[(int)good_matches[i].queryIdx].pt.x, 
      (int)keypoints_1[(int)good_matches[i].queryIdx].pt.y, 
      (int)keypoints_2[(int)good_matches[i].trainIdx].pt.x, 
      (int)keypoints_2[(int)good_matches[i].trainIdx].pt.y 
      ); 
    } */

    //RANSAC
    int max_iterations = (int)good_matches.size();
    int intlier_threshold = 5;
    int x_transform = 0;
    int y_transform = 0;
    int intlier_count = 0;
    int best_good_match = 0;
    for (int i = 0; i < max_iterations; i++) {
      int temp_x_transform = keypoints_1[good_matches[i].queryIdx].pt.x - keypoints_2[good_matches[i].trainIdx].pt.x
      int temp_y_transform = keypoints_1[good_matches[i].queryIdx].pt.y - keypoints_2[good_matches[i].trainIdx].pt.y
      int temp_count = 0;
      for (int j = 0; j < max_iterations; j++) {
	if (j == i)
	  continue
	int x_transform_diff = keypoints_1[good_matches[i].queryIdx].pt.x - temp_x_transform - keypoints_2[good_matches[i].trainIdx].pt.x;
	int y_transform_diff = keypoints_1[good_matches[i].queryIdx].pt.y - temp_y_transform - keypoints_2[good_matches[i].trainIdx].pt.y;
	if (x_transform_diff < 5 && y_transform_diff < 5)
	  temp_count = temp_count + 1;
      }
      if (temp_count > inlier_count){
	intlier_count = temp_count;
	x_transform = temp_x_transform;
	y_transform = temp_y_transform;
	best_good_match = i;
      }
    }
  }

  cv::SiftFeatureDetector sift_detector;
  cv::SiftDescriptorExtractor sift_extractor;
  std::vector<KeyPoints> sift_features( warped_inputs.size() );
  std::vector<cv::Mat> descriptors( warped_inputs.size() );
  for(int i = 0; i < warped_inputs.size(); i++) sift_detector.detect( warped_inputs[i].first, sift_features[i] );
  for(int i = 0; i < warped_inputs.size(); i++) sift_extractor.compute( warped_inputs[i].first, sift_features[i], descriptors[i] );

  // Perform a pairwise matching between images or incremental build-up
  
}
