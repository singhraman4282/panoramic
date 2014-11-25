#ifndef CAMERAVIDEOPUBLISHER_H
#define CAMERAVIDEOPUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <ctime>
#include <fstream>

namespace nurc {

class CameraVideoPublisher {
public:
  typedef sensor_msgs::SetCameraInfo::Request SetCameraInfoRequest;
  typedef sensor_msgs::SetCameraInfo::Response SetCameraInfoResponse;

  CameraVideoPublisher(std::string param_dir=std::string("~/Code/ros_ws/src/panoramic/config/"));

  ros::NodeHandle nh_;
  std::string param_dir_path_;

  void publishCameraImages();
  void publishCameraImages(const ros::TimerEvent& te);

  bool setCameraInfo(SetCameraInfoRequest& req, SetCameraInfoResponse& res);

private:
  std::vector<cv::VideoCapture> camera_video_capture_;
  std::vector<ros::Publisher> camera_image_publisher_;
  std::vector<ros::ServiceServer> camera_calib_service_;
};

} // namespace nurc

#endif // CAMERAVIDEOPUBLISHER_H
