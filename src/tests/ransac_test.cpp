#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <panoramic/SphericalStitch.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ransac_test");
  ros::NodeHandle nh;
  
  std::stringstream ss;
  std::string p_path = ros::package::getPath("panoramic");
  std::string first_path, second_path, third_path;
  ss << p_path << "/res/images/study_room_0.jpg";
  
  first_path = ss.str();
  ss.str(std::string());
  ss.clear();
  ss << p_path << "/res/images/study_room_1.jpg";
  second_path = ss.str();
  ss.str( std::string() );
  ss.clear();
  ss << p_path << "/res/images/study_room_2.jpg";
  third_path = ss.str();

  panoramic::SphericalStitch test_stitch;
  test_stitch.request.phi_res = test_stitch.request.theta_res = 1000;
  test_stitch.request.s = panoramic::SphericalStitchRequest::FOCAL_LENGTH;
  
  cv::Mat first_image = cv::imread( first_path.c_str() );
  cv::Mat second_image = cv::imread( second_path.c_str() );
  cv::Mat third_image = cv::imread( third_path.c_str() );
  
  if(!first_image.empty() && !second_image.empty()) {
    sensor_msgs::Image first_im, second_im, third_im;
    cv_bridge::CvImage cvi_first( std_msgs::Header(), std::string("bgr8"), first_image );
    cv_bridge::CvImage cvi_second( std_msgs::Header(), std::string("bgr8"), second_image );
    cv_bridge::CvImage cvi_third( std_msgs::Header(), std::string("bgr8"), third_image );
    cvi_first.toImageMsg( first_im );
    cvi_second.toImageMsg( second_im ); 
    cvi_third.toImageMsg( third_im );
    
    test_stitch.request.queue.push_back( first_im );
    test_stitch.request.queue.push_back( second_im );
    test_stitch.request.queue.push_back( third_im );

    // Define the camera model from camera calibration
    sensor_msgs::CameraInfo ci;
    ci.distortion_model = std::string("plumb_bob");
    ci.K[0] = 597.047010;
    ci.K[1] = 0.0;
    ci.K[1] = 325.596134;
    ci.K[1] = 0.0;
    ci.K[1] = 596.950842;
    ci.K[1] = 211.872049;
    ci.K[1] = 0.0;
    ci.K[1] = 0.0;
    ci.K[1] = 1.0;

    ci.D[0] = 0.013417;
    ci.D[1] = -0.105440;
    ci.D[2] = -0.016704;
    ci.D[3] = -0.002854;
    ci.D[4] = 0.0;
    test_stitch.request.camera_info = ci;
    
    ros::ServiceClient p_client = nh.serviceClient<panoramic::SphericalStitch>("stitch");
    if(p_client.call( test_stitch )) {
      ROS_INFO("Successfully called the service.");
    }
    else {
      ROS_ERROR("Panoramic server failed.");
    }
  }
  else {
    ROS_ERROR("Could not successfully open images.");
  }

  ros::spin();

}
