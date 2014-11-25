#ifndef PANORAMIC_H
#define PANORAMIC_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <panoramic/SphericalStitch.h>
#include <vector>

namespace nurc {
  
class Panoramic {
public:

  typedef panoramic::SphericalStitch::Request SphericalStitchRequest;
  typedef panoramic::SphericalStitch::Response SphericalStitchResponse;

  Panoramic();
  ~Panoramic();

  ros::NodeHandle nh_;
  ros::ServiceServer image_stitcher_;
  cv::Mat spherical_warping_;
  cv::Mat mapping_x_;
  cv::Mat mapping_y_;

  std::vector<cv::Mat> image_queue_;

  bool stitch(SphericalStitchRequest& req, SphericalStitchResponse& res);
  void generate_mapping_x();
  void generate_mapping_y();
  void generate_mapping();
  void generate_spherical_stitching();

private:
};

} // namespace nurc

#endif // PANORAMIC_H
