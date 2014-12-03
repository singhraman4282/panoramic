#ifndef PANORAMIC_H
#define PANORAMIC_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/stitching/warpers.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <panoramic/SphericalStitch.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <time.h>

namespace nurc {
  
struct SphericalTransform {
  SphericalTransform(int phi=0, int theta=0) : 
    phi_(phi), 
    theta_(theta) 
  {};

  std::string toString() {
    std::stringstream ss;
    ss << "[PHI: " << phi_ << ", THETA: " << theta_ << "]\n";
    return ss.str();
  };

  int phi_;
  int theta_;
};

bool DMatchDistanceCompare(cv::DMatch a, cv::DMatch b)
{
  return a.distance < b.distance;
}
  
class Panoramic {
public:

  typedef panoramic::SphericalStitch::Request SphericalStitchRequest;
  typedef panoramic::SphericalStitch::Response SphericalStitchResponse;
  typedef std::vector<cv::KeyPoint> KeyPoints;
  typedef std::pair<cv::Mat, cv::Mat> WarpedPair;
  typedef std::vector< std::vector<WarpedPair> > Pyramid;

  Panoramic();
  ~Panoramic();

  ros::NodeHandle nh_;
  ros::ServiceServer image_stitcher_;

  bool stitch(SphericalStitchRequest& req, SphericalStitchResponse& res);
  cv::Mat warp_to_hsphere(cv::Mat& input, int phi_res, int theta_res, int focal_length, cv::Mat& mask);
  void generate_image_transforms(cv::Mat& sphere, std::vector<WarpedPair>& warped_inputs, std::vector<SphericalTransform>& relative_transforms, int phi_res, int theta_res);
  void hsphere_to_sphere(WarpedPair& hsphere, cv::Mat& sphere, SphericalTransform& s_transform, int theta_res, int phi_res);
  void blend_sphere(std::vector<WarpedPair>& warped_inputs, cv::Mat& sphere, std::vector<SphericalTransform>& s_transforms, int phi_res, int theta_res);

};

} // namespace nurc

#endif // PANORAMIC_H
