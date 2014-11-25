#include <panoramic/CameraVideoPublisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_video_publisher");
  ros::NodeHandle nh;
  
  // Define the publisher and callback at 30 times a second
  nurc::CameraVideoPublisher cvp;
  ros::Timer image_publisher= nh.createTimer(ros::Duration(1./30.), boost::bind(&nurc::CameraVideoPublisher::publishCameraImages, &cvp, _1));

  ros::spin();

  return 0;
}
