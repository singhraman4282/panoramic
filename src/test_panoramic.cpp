#include <opencv2/stitching/warpers.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "panoramic");
  // ros::NodeHandle nh;
  
  cv::Mat test_image = cv::imread("/home/tgdiriba/Pictures/cat8.jpg");
  cv::Mat gpu_image_x = cv::Mat::ones(test_image.size(), CV_32FC1);
  cv::Mat gpu_image_y = cv::Mat::ones(test_image.size(), CV_32FC1);
  cv::Mat gpu_image = cv::Mat::zeros(test_image.size(), test_image.type());
  cv::remap(test_image, gpu_image, gpu_image_x, gpu_image_y, CV_INTER_LINEAR);
  cv::Mat warped_image = cv::Mat::zeros(test_image.size(), CV_8UC3);
  
  // Define the camera calibration matrices
  cv::Mat K(3,3,CV_32F);
  K.at<float>(0,0,0) = 430.21554970319971;
  K.at<float>(0,1,0) = 0.0;
  K.at<float>(0,2,0) = 306.6913434743704;
  K.at<float>(1,0,0) = 0.0;
  K.at<float>(1,1,0) = 430.53169252696676;
  K.at<float>(1,2,0) = 227.22480030078816;
  K.at<float>(2,0,0) = 0.0;
  K.at<float>(2,1,0) = 0.0;
  K.at<float>(2,2,0) = 1.0;
  
  cv::Mat R(3,3, CV_32F);
  R.at<float>(0,0,0) = 1.0;
  R.at<float>(0,1,0) = 0.0;
  R.at<float>(0,2,0) = 0.0;
  R.at<float>(1,0,0) = 0.0;
  R.at<float>(1,1,0) = 1.0;
  R.at<float>(1,2,0) = 0.0;
  R.at<float>(2,0,0) = 0.0;
  R.at<float>(2,1,0) = 0.0;
  R.at<float>(2,2,0) = 1.0;
  
  std::cout << K << std::endl;
  std::cout << R << std::endl;
  cv::detail::CylindricalWarper sw(500);
  // sw.warp(test_image, K, R, cv::INTER_LINEAR, cv::BORDER_CONSTANT, warped_image); 

  cv::namedWindow("original");
  cv::namedWindow("warped");

  cv::imshow("original", test_image);
  cv::imshow("warped", warped_image);

  cv::waitKey(0);
  
  /*ros::Rate r(1);
  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }*/

  return 0;
}
