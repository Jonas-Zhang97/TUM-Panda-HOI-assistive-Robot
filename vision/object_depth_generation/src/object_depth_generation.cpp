#include <object_depth_generation/object_depth_generation.h>

bool ObjectDepthGeneration::init()
{
  bounding_box_topic = "/darknet_ros/bounding_boxes";
  depth_image_topic = "/camera/aligned_depth_to_color/image_raw";

  bounding_boxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>(bounding_box_topic, 1, &ObjectDepthGeneration::boundingBoxesCallback, this);
  depth_image_sub_ = nh_.subscribe<sensor_msgs::Image>(depth_image_topic, 1, &ObjectDepthGeneration::depthImageCallback, this);

  object_depth_pub_ = nh_.advertise<sensor_msgs::Image>("/object_depth_image", 1);

  return true;
}

void ObjectDepthGeneration::update()
{
  objectDepthExtraction();
}

void ObjectDepthGeneration::objectDepthExtraction()
{
  cv::Mat objects_extracted_mat(720, 1280, CV_32FC1);
  for (int i = 0; i < boxes_num_; ++i)
  {
  }
  // ROS_INFO_STREAM("" << objects_extracted_mat.size);
}

void ObjectDepthGeneration::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  // convert bounding boxes to cv rectangle
  cv_bounding_boxes_.clear();
  for (const darknet_ros_msgs::BoundingBox& b_box : msg->bounding_boxes)
  {
    cv_bounding_boxes_.push_back(cv_bounding_box);
  }
  boxes_num_ = cv_bounding_boxes_.size();
}

void ObjectDepthGeneration::depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // save the depth image as a matrix
  depth_image_ptr_ = cv_bridge::toCvCopy(msg);
  depth_matrix_ = depth_image_ptr_->image;

  depth_matrix_size_.resize(2);
  depth_matrix_size_ = {depth_matrix_.rows, depth_matrix_.cols};
}