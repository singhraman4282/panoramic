#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <panoramic/Panoramic.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panoramic");
  ros::NodeHandle nh;

  nurc::Panoramic p_server;

  ros::spin();
}
