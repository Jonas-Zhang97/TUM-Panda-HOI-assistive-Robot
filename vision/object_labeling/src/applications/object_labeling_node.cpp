#include <object_labeling/object_labeling.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_labeling");
  
  ObjectLabeling ol;

  ol.init();

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ol.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}