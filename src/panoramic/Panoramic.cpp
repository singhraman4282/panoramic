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

void Panoramic::generate_spherical_stitch(cv::Mat& sphere, std::vector<WarpedPair>& warped_inputs, int phi_res, int theta_res)
{
  // Store all of the final transform in here

  std::vector<nurc::SphericalTransform> relative_transforms(warped_inputs.size() - 1);
  // Perform a pairwise matching between images or incremental build-up
  for(int i = 0; i < warped_inputs.size()-1; i++) {
    // Define references to images
    cv::Mat query_im = warped_inputs[i].first;
    cv::Mat train_im = warped_inputs[i+1].first;
    cv::Mat query_mask = warped_inputs[i].second;
    cv::Mat train_mask = warped_inputs[i+1].second;

    cv::SiftFeatureDetector sift_detector;
    KeyPoints query_kp, train_kp;
    sift_detector.detect( query_im, query_kp, query_mask);
    sift_detector.detect( train_im, train_kp, train_mask);

    cv::SiftDescriptorExtractor sift_features;
    cv::Mat query_features, train_features;
    sift_features.compute( query_im, query_kp, query_features );
    sift_features.compute( train_im, train_kp, train_features );

    cv::FlannBasedMatcher feature_matcher;
    std::vector<cv::DMatch> similar_features;
    feature_matcher.match( query_features, train_features, similar_features );

    double max_dist = 0, min_dist = 100;
    for(int f = 0; f < query_features.rows; f++) { 
      if( similar_features[f].distance < min_dist ) min_dist = similar_features[f].distance;
      if( similar_features[f].distance > max_dist ) max_dist = similar_features[f].distance;
    }

    std::vector<cv::DMatch> shared_features;
    for(int f = 0; f< query_features.rows; f++) { 
      if( similar_features[f].distance <= std::max( 2*min_dist, 0.02 ) )
        shared_features.push_back( similar_features[f] ); 
    }

    /* Displaying image and outputting some data

    &cv::Mat feature_im;
    drawMatches( query_im, query_kp, train_im, train_kp, shared_features, i
                 feature_im, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), 
                 DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Matched Features", feature_im );

    for( int i = 0; i < shared_features.size(); i++ ) { 
      printf( "-- Good Match Points [%d] 1 X-value: %d  1 Y-value: %d -- 2 X-value: %d 2 Y-value: %d \n", i, 
      (int)query_kp[ (int)shared_features[i].queryIdx ].pt.x, 
      (int)query_kp[ (int)shared_featuers[i].queryIdx ].pt.y, 
      (int)train_kp[ (int)shared_features[i].trainIdx ].pt.x, 
      (int)train_kp[ (int)shared_features[i].trainIdx ].pt.y 
      ); 
    } */

    //RANSAC
    // Define the maximum allowed deviation in terms of the Phi and Theta angles in radians
    double max_phi_dev = 5.*(M_PI/180.), max_theta_dev = 5.*(M_PI/180.);
    double error_threshold = pow( double(max_phi_dev*max_phi_dev + max_theta_dev*max_theta_dev), 0.5 );

    int max_trials = shared_features.size();  // Reduce if this turns out to be too large
    srand( time(NULL) );
    for(int j = 0; j < shared_features.size(); j++) {
      // Algorithm randomly select a pair
      // compute the transform from the train to query image
      // calculate the number of inliers
      // store that number and the transform in the best inlier variables
      // continue for the maximum number of iterations

      double max_inliers = 0;
      std::vector<cv::DMatch> r_features = shared_features;
      r_features.erase( r_features.begin() + i );
      for(int k = 0; k < max_trials; k++) {
        // Randomly pick a pair
        int rand_index = rand() % r_features.size();
        cv::DMatch rand_feature = r_features[rand_index];
        r_features.erase( r_features.begin() + rand_index );
        
        // Compute the transform
        int phi_t = query_kp[ r_features[rand_index].queryIdx ].pt.x - train_kp[ r_features[rand_index].trainIdx ].pt.x;
        int theta_t = query_kp[ r_features[rand_index].queryIdx ].pt.y - train_kp[ r_features[rand_index].trainIdx ].pt.y;
        nurc::SphericalTransform trial_transform( phi_t, theta_t );
        
        // Calculate the number of inliers using the computed transform
        // If the number of inliers is greater than the previous transform replace
        int trial_inliers = 0;
        for(int l = 0; l < r_features.size(); l++) {
          int trial_phi = query_kp[ r_features[l].queryIdx ].pt.x - train_kp[ r_features[l].trainIdx ].pt.x;
          int trial_theta = query_kp[ r_features[l].queryIdx ].pt.y - train_kp[ r_features[l].trainIdx ].pt.y;
          double transform_error = pow( double(trial_phi*trial_phi + trial_theta*trial_theta), 0.5 );
          if(transform_error < error_threshold) trial_inliers++;
        }

        if(trial_inliers > max_inliers) {
          max_inliers = trial_inliers;
          relative_transforms[i] = trial_transform;
        }
      }

    }
  }
}
