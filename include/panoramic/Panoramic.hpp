#ifndef PANORAMIC_H
#define PANORAMIC_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <panoramic/SphericalStitch.h>
#include <vector>
#include <cmath>
#include <algorithm>

namespace nurc {
  
class Panoramic {
public:

  typedef panoramic::SphericalStitch::Request SphericalStitchRequest;
  typedef panoramic::SphericalStitch::Response SphericalStitchResponse;
  typedef std::vector<cv::KeyPoint> KeyPoints;

  Panoramic();
  ~Panoramic();

  ros::NodeHandle nh_;
  ros::ServiceServer image_stitcher_;

  bool stitch(SphericalStitchRequest& req, SphericalStitchResponse& res);
  cv::Mat map_to_sphere(cv::Mat& input, int phi_res, int theta_res, int focal_length, cv::Mat& mask);
  void generate_spherical_stitch(cv::Mat& sphere, std::vector< std::pair<cv::Mat, cv::Mat> >& warped_inputs, int phi_res, int theta_res);

private:
  // Camera Calibration
};

} // namespace nurc

#endif // PANORAMIC_H
