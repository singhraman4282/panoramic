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
  ROS_INFO("Received stitch request. Generating warped images.");

  double s;
  int theta_res, phi_res;
  if(req.phi_res == SphericalStitchRequest::MAX_RES)
    phi_res = (int)round(double( 2. * M_PI ) / atan( 1. /s ) );
  else phi_res = req.phi_res;
  if(req.theta_res == SphericalStitchRequest::MAX_RES)
    theta_res = (int)round(double( 2. * M_PI ) / atan( 1. / s ) );
  else theta_res = req.theta_res;
  if(req.s == SphericalStitchRequest::FOCAL_LENGTH)
    s = 500;
  else s = req.s;

  cv::Mat sphere( phi_res, 2*theta_res, CV_8UC3 );
  std::vector<WarpedPair> image_queue;
  for( int i = 0; i < req.queue.size(); i++ ) {
    cv_bridge::CvImagePtr input_image = cv_bridge::toCvCopy( req.queue[i], sensor_msgs::image_encodings::BGR8 );
    
    cv::Mat input_uncalibrated = input_image->image.clone();
    /*cv::Mat input_calibrated;
    cv::Mat K(3, 3, CV_64FC3);
    cv::Mat D(1, 5, CV_64FC3);
    // Fix any spherical distortion in the image using the camera information
    if(std::string("plumb_bob") == req.camera_info.distortion_model) {
      K.at<double>(0,0) = req.camera_info.K[0];
      K.at<double>(0,1) = req.camera_info.K[1];
      K.at<double>(0,2) = req.camera_info.K[2];
      K.at<double>(1,1) = req.camera_info.K[3];
      K.at<double>(1,2) = req.camera_info.K[4];
      K.at<double>(1,3) = req.camera_info.K[5];
      K.at<double>(2,1) = req.camera_info.K[6];
      K.at<double>(2,2) = req.camera_info.K[7];
      K.at<double>(2,3) = req.camera_info.K[8];
      D.at<double>(0,0) = req.camera_info.D[0];
      D.at<double>(0,1) = req.camera_info.D[1];
      D.at<double>(0,2) = req.camera_info.D[2];
      D.at<double>(0,3) = req.camera_info.D[3];
      D.at<double>(0,4) = req.camera_info.D[4];
      // cv::undistort(input_uncalibrated, input_calibrated, K, D);
    }
    else input_calibrated = input_image->image;*/

    cv::Mat warp, mask( phi_res, theta_res, CV_8UC1, cv::Scalar(0) );
    warp = warp_to_hsphere( input_uncalibrated, phi_res, theta_res, s, mask );
    image_queue.push_back( WarpedPair( warp, mask ) );
  }

  ROS_INFO("Successufully generated warped images.");
  ROS_INFO("Generating spherical stitch.");

  std::vector<SphericalTransform> s_transforms;
  generate_image_transforms(sphere, image_queue, s_transforms, phi_res, theta_res);
  
  SphericalTransform t(0,0);
  for(int i = 0; i < image_queue.size(); i++) {
    hsphere_to_sphere(image_queue[i], sphere, t, theta_res, phi_res);
    if(i < s_transforms.size()) {
      t.phi_ += s_transforms[i].phi_;
      t.theta_ += s_transforms[i].theta_;
    }
  }

  blend_sphere(image_queue, sphere, s_transforms, phi_res, theta_res);

  std::string p_path = ros::package::getPath("panoramic");
  std::stringstream ss;
  ss << p_path << "/res/images/mask.jpg";
  cv::imwrite(ss.str().c_str(), image_queue[0].second);
  
  ss.clear();
  ss.str( std::string() );
  ss << p_path << "/res/images/sphere.jpg";
  cv::imwrite(ss.str().c_str(), sphere);
  
  return true;
}

/*void Panoramic::generated_reflected_images(std::vector<WarpedPair>& warped_inputs, std::vector<WarpedPair>& reflected, int expansion)
{
  for(int i = 0; i < warped_inputs.size(); i++) {
    cv::Mat expanded_image, expanded_mask;
  }
}*/

void Panoramic::blend_sphere(std::vector<WarpedPair>& warped_inputs, cv::Mat& sphere, std::vector<SphericalTransform>& s_transforms, int phi_res, int theta_res)
{
  // OpenCV Image Blending
  // Maximum size at the boundaries is given by max image size
  cv::Mat expanded_sphere( 3 * phi_res, 4 * theta_res, CV_8UC3 );
  cv::detail::MultiBandBlender mbb(false, 100);
  int esc_phi = expanded_sphere.rows/2;
  int esc_theta = expanded_sphere.cols/2;
  SphericalTransform t(0,0);
  mbb.prepare( cv::Rect( 0, 0, expanded_sphere.rows-1, expanded_sphere.cols-1 ) );
  for(int i = 0; i < warped_inputs.size(); i++) {
    // Using the spherical transforms compute all of the corners in order
    int wic_phi = warped_inputs[i].first.rows/2;
    int wic_theta = warped_inputs[i].first.cols/2;
    int tl_y, tl_x;
    tl_y = t.phi_-wic_phi+esc_phi;
    tl_x = t.theta_-wic_theta+esc_theta;
    ROS_INFO("%d %d", tl_y, tl_x);
    mbb.feed( warped_inputs[i].first, warped_inputs[i].second, cv::Point(tl_x, tl_y) );

    if(i < s_transforms.size()) {
      t.phi_ += s_transforms[i].phi_;
      t.theta_ += s_transforms[i].theta_;
    }
  }
  cv::Mat o = cv::Mat::ones( expanded_sphere.rows, expanded_sphere.cols, CV_8UC1 );
  mbb.blend( expanded_sphere, o );

  std::string p_path = ros::package::getPath("panoramic");
  std::stringstream ss;
  ss << p_path << "/res/images/expanded_sphere.jpg";
  cv::imwrite(ss.str().c_str(), expanded_sphere);

}

cv::Mat Panoramic::warp_to_hsphere(cv::Mat& input, int phi_res, int theta_res, int focal_length, cv::Mat& mask)
{
  // Warp planar image to half-sphere with resolution given by inputs

  // Manage the case in which planar image is too large 
  // for the sphere and the given resolution resulting in gaps
  double d_theta = atan(1.) / double(focal_length);
  double d_phi = atan(1. / double(focal_length) );
  double R_theta = d_theta * theta_res / M_PI;
  double R_phi = d_phi * phi_res / M_PI;
  int R = ceil(std::max( R_theta, R_phi ));
  
  // Scaling of the input image will correct issues with incompatible sizes
  cv::Mat scaled_input;
  cv::resize( input, scaled_input, cv::Size( ceil(R)*input.rows, ceil(R)*input.cols ) );
  
  // Warped image will be indexed using phi and theta coordinates
  cv::Mat warp( phi_res, theta_res, CV_8UC3 );
  // mask = cv::Mat::zeros( phi_res, theta_res, CV_8UC1 );
  for(int y = 0; y < scaled_input.rows; y++) {
    for(int x = 0; x < scaled_input.cols; x++) {
      // Compute transforms
      double y_disp = double(y-scaled_input.rows/2)/double(R);
      double x_disp = double(x-scaled_input.cols/2)/double(R);
      int phi = ( atan( y_disp / pow( pow( x_disp, 2 ) + pow( double(focal_length), 2 ) , 0.5 ) ) + M_PI/2. ) * double( phi_res ) / M_PI;
      int theta = atan( x_disp / double(focal_length) ) * double( theta_res ) / M_PI;
      if(theta >= theta_res/2)
        theta = theta-theta_res/2;
      else
        theta = theta+theta_res/2;

      // Map
      warp.at<cv::Vec3b>(phi, theta) = scaled_input.at<cv::Vec3b>(y, x);
      mask.at<unsigned char>(phi, theta) = 255;
    }
  } 

  static int sn = 0;
  sn++;
  std::string p_path = ros::package::getPath("panoramic");
  std::stringstream ss;
  ss << p_path << "/res/images/stitched_" << sn << ".jpg";
  cv::imwrite(ss.str().c_str(), warp);
  
  return warp;
}

void Panoramic::hsphere_to_sphere(WarpedPair& hsphere, cv::Mat& sphere, SphericalTransform& s_transform, int theta_res, int phi_res)
{
  int hc_theta = hsphere.first.cols/2;
  int hc_phi = hsphere.first.rows/2;
  int s_theta = sphere.cols/2;
  int s_phi = sphere.rows/2;
  for(int phi_index = 0; phi_index < hsphere.first.rows; phi_index++) {
    for(int theta_index = 0; theta_index < hsphere.first.cols; theta_index++) {
      // Compute the shifts that have compensated for out-of-bounds issues
      int phi_shift = (s_transform.phi_+phi_index-hc_phi+s_phi) % sphere.rows;
      int theta_shift = (s_transform.theta_+theta_index-hc_theta+s_theta) % sphere.cols;
      if(phi_shift < 0)
        phi_shift += sphere.rows;
      else if(phi_shift >= sphere.rows)
        phi_shift = sphere.rows - phi_shift;
      if(theta_shift < 0)
        theta_shift += sphere.cols;
      else if(theta_shift >= sphere.cols)
        theta_shift = sphere.cols - theta_shift;

      // Check the mask to make sure that this is part of the warped image and assign if so
      if(hsphere.second.at<unsigned char>(phi_index, theta_index) != 0) {
        sphere.at<cv::Vec3b>(phi_shift, theta_shift) = hsphere.first.at<cv::Vec3b>(phi_index, theta_index);
      }
    }
  }
}

void Panoramic::generate_image_transforms(cv::Mat& sphere, std::vector<WarpedPair>& warped_inputs, std::vector<SphericalTransform>& relative_transforms, int phi_res, int theta_res)
{
  ROS_INFO("Beginning spherical stitch generation.");

  // Store all of the final transform in here
  relative_transforms.resize(warped_inputs.size() - 1);
  // Perform a pairwise matching between images or incremental build-up
  for(int i = 0; i < warped_inputs.size()-1; i++) {
    ROS_INFO("Matching images %d and %d.", i, i+1);

    // Define references to images
    cv::Mat& query_im = warped_inputs[i].first;
    cv::Mat& train_im = warped_inputs[i+1].first;
    cv::Mat& query_mask = warped_inputs[i].second;
    cv::Mat& train_mask = warped_inputs[i+1].second;

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
    min_dist = std::min_element( similar_features.begin(), similar_features.end(), nurc::DMatchDistanceCompare )->distance;
    std::vector<cv::DMatch> shared_features;
    for(int f = 0; f< query_features.rows; f++) { 
      if( similar_features[f].distance <= std::max( 20*min_dist, 0.02 ) )
        shared_features.push_back( similar_features[f] ); 
    }

    //RANSAC
    // Define the maximum allowed deviation in terms of the Phi and Theta angles in radians
    ROS_INFO("Beginning RANSAC");
    double max_phi_dev = 5.*(M_PI/180.), max_theta_dev = 5.*(M_PI/180.);
    double error_threshold = pow( double(max_phi_dev*max_phi_dev + max_theta_dev*max_theta_dev), 0.5 );

    int max_trials = shared_features.size();  // Reduce if this turns out to be too large
    srand( time(NULL) );
    for(int j = 0; j < shared_features.size()-1; j++) {
      ROS_INFO("Computing random feature transform index %d...", j);
      // Initilize inlier tracking
      int max_inliers = 0;
      std::vector<cv::DMatch> r_features = shared_features;
      r_features.erase( r_features.begin() + i );

      int max_runs = std::min(max_trials, (int)r_features.size());
      for(int k = 0; k < max_runs; k++) {
        // Randomly pick a pair
        int rand_index = rand() % r_features.size();
        cv::DMatch rand_feature = r_features[rand_index];
        r_features.erase( r_features.begin() + rand_index );
        
        // Compute the transform
        int theta_t = query_kp[ (int)rand_feature.queryIdx ].pt.x - train_kp[ (int)rand_feature.trainIdx ].pt.x;
        int phi_t = query_kp[ (int)rand_feature.queryIdx ].pt.y - train_kp[ (int)rand_feature.trainIdx ].pt.y;
        nurc::SphericalTransform trial_transform( phi_t, theta_t );
        
        // Calculate the number of inliers using the computed transform
        // If the number of inliers is greater than the previous transform replace
        int trial_inliers = 0;
        for(int l = 0; l < r_features.size(); l++) {
          int trial_theta = query_kp[ (int)r_features[l].queryIdx ].pt.x - train_kp[ (int)r_features[l].trainIdx ].pt.x;
          int trial_phi = query_kp[ (int)r_features[l].queryIdx ].pt.y - train_kp[ (int)r_features[l].trainIdx ].pt.y;

          double theta_error = double(trial_theta - theta_t)*(M_PI/double(theta_res));
          double phi_error = double(trial_phi - phi_t)*(M_PI/double(phi_res));
          double transform_error = pow( double(phi_error*phi_error + theta_error*theta_error), 0.5 );
          if(transform_error < error_threshold) trial_inliers++;
        }
        if(trial_inliers > max_inliers) {
          ROS_INFO("Resetting relative transforms.");
          max_inliers = trial_inliers;
          relative_transforms[i] = trial_transform;
        }
      }
    }
  }

  ROS_INFO("Successfully computed relative transforms.");
  ROS_INFO("Relative transforms: %d", relative_transforms.size());
  for(int t = 0; t < relative_transforms.size(); t++) {
    std::cout << relative_transforms[t].toString() << std::endl;
  }
}
