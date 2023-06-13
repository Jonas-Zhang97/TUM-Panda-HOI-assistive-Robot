#include <arm_motion/grasp.h>

int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "motion_control_node");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Create objects to get access to my libraries
  Grasp grasp;

  // Set parameters for Rviz
  namespace rvt = rviz_visual_tools;

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  grasp.visual_tools.loadRemoteControl();

  // Initialize the scene in Rviz
  grasp.visual_tools.deleteAllMarkers();
  grasp.PSI.removeCollisionObjects(grasp.PSI.getKnownObjectNames());
  grasp.visual_tools.publishText(text_pose, "Motion Visualization", rvt::WHITE, rvt::XLARGE);
  grasp.visual_tools.trigger();

  // Reset Karina position
  grasp.grasp();
  // End the node
  ros::shutdown();
  return 0;
}
