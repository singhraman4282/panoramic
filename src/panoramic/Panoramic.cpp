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

  cv::SiftFeatureDetector sift_detector;
  cv::SiftDescriptorExtractor sift_extractor;
  std::vector<KeyPoints> sift_features( warped_inputs.size() );
  std::vector<cv::Mat> descriptors( warped_inputs.size() );
  for(int i = 0; i < warped_inputs.size(); i++) sift_detector.detect( warped_inputs[i].first, sift_features[i] );
  for(int i = 0; i < warped_inputs.size(); i++) sift_extractor.compute( warped_inputs[i].first, sift_features[i], descriptors[i] );

  // Perform a pairwise matching between images or incremental build-up
  
}
