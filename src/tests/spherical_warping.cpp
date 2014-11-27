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

  std::cout << "Reading images.\n";
  cv::Mat sample = cv::imread("/home/tgdiriba/Code/ros_ws/src/panoramic/res/images/fuji.jpg");

  nurc::Panoramic p_server;
  cv::Mat warped = p_server.map_to_sphere(sample, 1000, 1000, 100);

  cv::imshow("Fuji", sample);
  cv::imshow("Fuji Warped", warped);

  cv::waitKey(0);

  return 0;
}
