#include <panoramic/Panoramic.hpp>
#include <iostream>

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
  std::vector<cv::Mat> image_queue;
  for( int i = 0; i < req.queue.size(); i++ ) {
    cv_bridge::CvImagePtr input_image = cv_bridge::toCvCopy( req.queue[i], sensor_msgs::image_encodings::BGR8 );
    image_queue.push_back( map_to_sphere( input_image->image, phi_res, theta_res, focal_length ) );
  }

  generate_spherical_stitch(sphere, image_queue, phi_res, theta_res);
  
  return true;
}

cv::Mat Panoramic::map_to_sphere(cv::Mat& input, int phi_res, int theta_res, int focal_length)
{
  // Half-warp warping
  // Total resolution has to be an even number
  cv::Mat warp;

  // Testing OpenCV Spherical Warping
  cv::detail::SphericalWarper sw(focal_length);
  // float K_data[9] = { 597.0470, 0, 325.596134, 0, 596.950842, 211.872049, 0, 0, 1 };
  cv::Mat K(3,3, CV_32FC1);
  /*K.at<float>(0,0) = 597.0470;
  K.at<float>(1,0) = 0;
  K.at<float>(2,0) = 325.696134;
  K.at<float>(0,1) = 0;
  K.at<float>(1,1) = 596;
  K.at<float>(2,1) = 211.872049;
  K.at<float>(0,2) = 0;
  K.at<float>(1,2) = 0;
  K.at<float>(2,2) = 1;*/

  cv::Mat R = cv::Mat::eye(3,3, CV_32FC1);
  sw.warp(input, K, R, CV_INTER_LINEAR, cv::BORDER_CONSTANT, warp);

  return warp;
}

void Panoramic::generate_spherical_stitch(cv::Mat& sphere, std::vector<cv::Mat>& warped_inputs, int phi_res, int theta_res)
{
  // Algorithm
  // Select the first image to be the center
  // Incrementally add elements to the spherial warping
  //   Each time an element is added run an image alignment over it

  cv::SiftFeatureDetector sift_detector;
  cv::SiftDescriptorExtractor sift_extractor;
  std::vector<KeyPoints> sift_features( warped_inputs.size() );
  std::vector<cv::Mat> descriptors( warped_inputs.size() );
  for(int i = 0; i < warped_inputs.size(); i++) sift_detector.detect( warped_inputs[i], sift_features[i] );
  for(int i = 0; i < warped_inputs.size(); i++) sift_extractor.compute( warped_inputs[i], sift_features[i], descriptors[i] );

  // Perform a pairwise matching between images or incremental build-up
  
}
