#include <object_depth_generation/object_depth_generation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_depth_generation_node");

  ObjectDepthGeneration odg;

  odg.init();

  ros::Rate loop_rate(10); 

  while(ros::ok())
  {
    odg.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}