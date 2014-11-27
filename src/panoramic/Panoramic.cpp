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
  cv::Mat warp( phi_res, theta_res, CV_8UC3 );

  for(int y = 0; y < input.rows; y++) {
    for(int x = 0; x < input.cols; x++) {
      int phi = ( atan( double(y) / pow( pow( double(x), 2 ) + pow( double(focal_length), 2 ) , 0.5 ) )
                + M_PI/2. ) * double( phi_res ) / ( 2. * M_PI );
      int theta = atan( double(x) / double(focal_length) ) * double( theta_res ) / ( 2. * M_PI );
      // std::cout << phi << " " << theta << std::endl;
      warp.at<cv::Vec3b>(phi, theta) = input.at<cv::Vec3b>(y, x);
    }
  } 

  return warp;
}

void Panoramic::generate_spherical_stitch(cv::Mat& sphere, std::vector<cv::Mat>& warped_inputs, int phi_res, int theta_res)
{
  // Algorithm
  // Select the first image to be the center
  // Incrementally add elements to the spherial warping
  //   Each time an element is added run an image alignment over it
  
  
}
