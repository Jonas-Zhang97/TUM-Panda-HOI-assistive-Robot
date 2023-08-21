#include <depth_image_converter/depth_image_converter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_converter_node");

  DepthImageConverter dic;

  if (!dic.init())
  {
    ROS_ERROR_STREAM("Initialization failed");
    return 0;
  }
  else
  {
    ROS_INFO_STREAM("Initialized: depth_image_converter_node");
  }

  ros::Rate loop_rate(1); 

  while(ros::ok())
  {
    dic.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}