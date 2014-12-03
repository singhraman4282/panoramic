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

void Panoramic::generate_spherical_stitch(cv::Mat& sphere, WarpedPairs& warped_inputs, int phi_res, int theta_res)
{
  // Perform a pairwise matching between images or incremental build-up
  for(int i = 0; i < warped_inputs.size()-1; i++) {
    // Define references to images
    cv::Mat query_image = warped_inputs[i].first;
    cv::Mat train_image = warped_inputs[i+1].first;
    cv::Mat query_mask = warped_inputs[i].second;
    cv::Mat train_mask = warped_inputs[i+1].second;

    cv::SiftFeatureDetector sift_detector;
    KeyPoints query_kp, train_kp;
    sift_detector.detect( query_image, query_kp, query_mask);
    sift_detector.detect( second_image, trian_kp, train_mask);

    cv::SiftDescriptorExtractor sift_features;
    cv::Mat query_features, train_features;
    sift_features.compute( query_im, query_im, query_features );
    sift_features.compute( train_im, train_kp, train_features );

    cv::FlannBasedMatcher feature_matcher;
    std::vector<cv::DMatch> similar_features;
    matcher.match( query_features, train_features, similar_features );

    double max_dist = 0, min_dist = 100;
    for( int i = 0; i < query_features.rows; i++ ) { 
      if( similar_features[i].distance < min_dist ) min_dist = similar_features[i].distance;
      if( similar_features[i].distance > max_dist ) max_dist = similar_features[i].distance;
    }

    std::vector<cv::DMatch> shared_features;
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
      if( similar_features[i].distance <= std::max(2*min_dist, 0.02) )
        shared_features.push_back( similar_features[i]); 
    }

    /* Displaying image and outputting some data

    &cv::Mat feature_im;
    drawMatches( query_im, query_kp, train_im, train_kp, shared_features, i
                 feature_im, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), 
                 DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Matched Features", img_matches );

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
      int temp_x_transform = query_kp[ shared_features[i].queryIdx ].pt.x - train_kp[ shared_features[i].trainIdx ].pt.x
      int temp_y_transform = query_kp[ shared_features[i].queryIdx ].pt.y - train_kp[ shared_features[i].trainIdx ].pt.y
      int temp_count = 0;
      for (int j = 0; j < max_iterations; j++) {
	if (j == i)
	  continue
	int x_transform_diff = query_kp[ shared_features[i].queryIdx ].pt.x - temp_x_transform - train_kp[ shared_features[i].trainIdx ].pt.x;
	int y_transform_diff = query_kp[ shared_features[i].queryIdx ].pt.y - temp_y_transform - train_kp[ shared_features[i].trainIdx ].pt.y;
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
}
