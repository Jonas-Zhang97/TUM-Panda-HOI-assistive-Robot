#include <object_labeling/object_labeling.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;

  std::string objects_cloud_topic = "/objects_cloud";  
  std::string camera_info_topic = "/camera/color/camera_info";   // So weird...
  std::string camera_frame = "camera_depth_optical_frame"; 

  ObjectLabeling labeling(
    objects_cloud_topic,
    camera_info_topic,
    camera_frame);

  // Init
  if(!labeling.initalize(nh))
  {
    return -1;
  }

  // Run
  ros::Rate rate(30);
  while(ros::ok())
  {
    labeling.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
