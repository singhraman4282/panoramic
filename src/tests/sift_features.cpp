#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sift_features");
  ros::NodeHandle nh;

  cv::Mat sample = cv::imread("/home/tgdiriba/Code/ros_ws/src/panoramic/res/images/fuji.jpg");

  cv::SIFT sf(1000);
  std::vector<cv::KeyPoint> kp;
  sf(sample, cv::noArray(), kp, cv::noArray());
  cv::drawKeypoints(sample, kp, sample);

  cv::imshow("sift_features", sample);

  cv::waitKey(10000);

  return 0;
}
