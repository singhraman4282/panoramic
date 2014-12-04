#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <panoramic/SphericalStitch.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stitching_images");
  ros::NodeHandle nh;

  std::stringstream ss;
  std::string p_path = ros::package::getPath("panoramic");
  std::cout << "Please input the paths to the images to stitch in order.\n";
  std::cout << "The paths should start in the panoramic package res/images directory.\n";

  bool finished = false;
  std::string resp;
  std::vector<sensor_msgs::Image> images;
  while(!finished) {
    std::cout << "Path to image " << images.size() << ": ";
    ss.clear();
    ss.str( std::string() );
    std::string im_path;
    std::cin >> im_path;
    ss << p_path << "/res/images/" << im_path;
    
    cv::Mat im = cv::imread(ss.str().c_str());
    if(!im.empty()) {
      cv_bridge::CvImage cvi(std_msgs::Header(), std::string("bgr8"), im);
      sensor_msgs::Image img_msg;
      cvi.toImageMsg( img_msg );
      images.push_back( img_msg );

      std::cout << "Continue (y/n)? ";
      std::cin >> resp;
      if(resp != "y") finished = true;
    }
    else {
      std::cout << "Invalid image encountered.\n";
      finished = true;
    }
  }

  panoramic::SphericalStitch stitch_srv;

  // Defining camera calibration information
  sensor_msgs::CameraInfo ci;
  ci.distortion_model = std::string("plumb_bob");
  ci.K[0] = 597.047010;
  ci.K[1] = 0.0;
  ci.K[2] = 325.596134;
  ci.K[3] = 0.0;
  ci.K[4] = 596.950842;
  ci.K[5] = 211.872049;
  ci.K[6] = 0.0;
  ci.K[7] = 0.0;
  ci.K[8] = 1.0;

  ci.D.resize(5);
  ci.D[0] = 0.013417;
  ci.D[1] = -0.105440;
  ci.D[2] = -0.016704;
  ci.D[3] = -0.002854;
  ci.D[4] = 0.0;
  stitch_srv.request.camera_info = ci;

  stitch_srv.request.s = panoramic::SphericalStitchRequest::FOCAL_LENGTH;
  stitch_srv.request.phi_res = stitch_srv.request.theta_res = 1000;
  stitch_srv.request.queue = images;

  ros::ServiceClient sc = nh.serviceClient<panoramic::SphericalStitch>("stitch");
  if(sc.call(stitch_srv)) {
    // Save stitched images
    std::cout << "Successfully stitched images.\n";
    std::cout << "What do you want to name the spherical stitch? ";
    std::string stitch_path;
    std::cin >> stitch_path;
    ss.clear();
    ss.str( std::string() );
    ss << p_path << "/res/images/" << stitch_path;
    
    cv_bridge::CvImagePtr mat_im = cv_bridge::toCvCopy(stitch_srv.response.stitch, sensor_msgs::image_encodings::BGR8);

    // Convert to cv::Mat and save
    cv::imwrite(ss.str().c_str(), mat_im->image);
    std::cout << "Successfully saved spherical stitch.\nFinished program.\n";
  }
  else {
    std::cout << "Stitch server failed.\n";
  }
}
