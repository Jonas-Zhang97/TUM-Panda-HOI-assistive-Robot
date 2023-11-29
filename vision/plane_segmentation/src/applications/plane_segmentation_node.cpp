#include <plane_segmentation/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");

  PlaneSegmentation ps;

  ps.init();

  ros::Rate loop_rate(0.5);
  while (ros::ok())
  {
    ps.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}