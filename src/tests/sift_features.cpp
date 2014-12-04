#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sift_features");
  ros::NodeHandle nh;

  cv::Mat sample = cv::imread("/home/parallels/Pictures/Tech_tester.jpg");

  cv::SurfFeatureDetector sfd(300);

  cv::Mat descriptor;

  std::vector<cv::KeyPoint> kp;
  sfd.detect(sample, kp);
  
  cv::SurfDescriptorExtractor sde;
  sde.compute( sample, kp, descriptor );
  
  cv::drawKeypoints(sample, kp, sample);

  cv::imshow("sift_features", sample);

  cv::waitKey(10000);

  return 0;
}
