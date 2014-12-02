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
  std::string first_path, second_path, output_path;
  std::cout << "Please input the path to the first image: ";
  std::cin >> first_path;
  std::cout << "Please input the path to the second image: ";
  std::cin >> second_path;
  std::cout << "Please input the path to where to save the matched image: ";
  std::cin >> output_path;
  std::cout << "Please input the desired focal length: ";
  std::cin >> focal_length;
  std::cout << std::endl;
  
  std::cout << "Reading images.\n";
  cv::Mat first_sample = cv::imread(first_input_path.c_str());
  cv::Mat second_sample = cv::imread(second_input_path.c_str());

  nurc::Panoramic p_server;
  cv::Mat first_mask, second_mask;
  cv::Mat first_warped = p_server.map_to_sphere(first_sample, 1000, 1000, focal_length, first_mask);
  cv::Mat second_warped = p_server.map_to_sphere(second_sample, 1000, 1000, focal_length, second_mask);
  
  cv::imshow(first_path.c_str(), first_sample);
  cv::imshow(second_path.c_str(), second_sample);
  cv::imshow("First Warped", first_warped);
  cv::imshow("Second Warped", second_warped);

  // Modify for FLANN code
  cv::imwrite(output_path.c_str(), warped);
  
  cv::waitKey(0);
  
  return 0;
}
