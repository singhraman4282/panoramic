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

  double focal_length;
  std::string input_path, output_path;
  std::cout << "Please input the path to the image: ";
  std::cin >> input_path;
  std::cout << "Please input the combined path and filename of the warped image: ";
  std::cin >> output_path;
  std::cout << "Please input the desired focal length: ";
  std::cin >> focal_length;
  std::cout << std::endl;
  
  std::cout << "Reading image.\n";
  cv::Mat sample = cv::imread(input_path.c_str());
  cv::Mat mask(1000,1000,CV_8UC1);

  nurc::Panoramic p_server;
  cv::Mat warped = p_server.warp_to_hsphere(sample, 1000, 1000, focal_length, mask);
  
  cv::imshow(input_path.c_str(), sample);
  cv::imshow("Warped", warped);

  cv::imwrite(output_path.c_str(), warped);
  std::cout << "Press any key to exit.";
  
  cv::waitKey(0);
  
  return 0;
}
