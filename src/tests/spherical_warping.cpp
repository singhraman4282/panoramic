#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <panoramic/Panoramic.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spherical_warping");
  ros::NodeHandle nh;
  
  cv::Mat sample = cv::imread("/home/parallels/Pictures/Tech_tester.jpg");
  nurc::Panoramic p_server;
  cv::Mat mask;
  cv::Mat warped = p_server.warp_to_hsphere(sample, 1000, 1000, 1050, mask);
  cv::imshow("Warped", warped);

  cv::imwrite("/home/parallels/Pictures/Tech_tester_warped.jpg", warped);
  
  cv::waitKey(0);
  
  return 0;
}
