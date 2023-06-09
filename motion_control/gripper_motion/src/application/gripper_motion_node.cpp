#include <gripper_motion/gripper_motion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_motion_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);

  spinner.start();

  GripperMotion gripper_motion;

  gripper_motion.init();

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    gripper_motion.update();
    loop_rate.sleep();
  }

  return 0;
}