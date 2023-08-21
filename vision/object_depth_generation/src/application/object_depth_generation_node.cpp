#include <object_depth_generation/object_depth_generation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_depth_generation_node");

  ObjectDepthGeneration odg;

  // 10 sec delay to wait for the previous initializations
  // ros::Duration(30.0).sleep();

  if (!odg.init())
  {
    return 0;
  }
  else
  {
    ROS_INFO_STREAM("Initialized: object_depth_generation_node");
  }

  ros::Rate loop_rate(0.5); 

  while(ros::ok())
  {
    odg.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}