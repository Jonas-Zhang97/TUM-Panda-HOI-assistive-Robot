#include <object_depth_generation/object_depth_generation.h>

bool ObjectDepthGeneration::init()
{
  bounding_box_topic = "/darknet_ros/bounding_boxes";

  std::string input_image_topic;

  nh_.getParam("/object_depth_generation_node/topic", input_image_topic);


  bounding_boxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>(bounding_box_topic, 1, &ObjectDepthGeneration::boundingBoxesCallback, this);
  depth_image_sub_ = nh_.subscribe<sensor_msgs::Image>(input_image_topic, 1, &ObjectDepthGeneration::depthImageCallback, this);
  target_name_sub_ = nh_.subscribe<std_msgs::String>("/hoi/target_object_name", 1, &ObjectDepthGeneration::targetNameCallback, this);

  object_depth_pub_ = nh_.advertise<sensor_msgs::Image>("/hoi/depth_image", 1);

  cv_type_map_["16UC1"] = CV_16UC1;

  has_boxes_ = false;
  has_image_ = false;
  has_command_ = false;

  return true;
}

void ObjectDepthGeneration::update()
{
  if (has_boxes_ && has_image_ && has_command_)
  {
    boxSelection();
    objectDepthExtraction();
    fromMatToMsg();
    
    // reset the flags
    has_boxes_ = false;
    has_image_ = false;
    has_command_ = false;
  }
}

int ObjectDepthGeneration::boxSelection()
{
  for (const darknet_ros_msgs::BoundingBox& b_box : boxes_.bounding_boxes)
  {
    if (b_box.Class == target_name_)
    {
      cv_bounding_box_.x = b_box.xmin;
      cv_bounding_box_.y = b_box.ymin;
      cv_bounding_box_.width = b_box.xmax-b_box.xmin;
      cv_bounding_box_.height = b_box.ymax-b_box.ymin;
      ROS_INFO_STREAM("cv bounding box: " << cv_bounding_box_);
    }
  }

  return 1;
}

void ObjectDepthGeneration::objectDepthExtraction()
{
  // create an empty mat
  objects_extracted_mat_.create(depth_matrix_size_[0], depth_matrix_size_[1], cv_depth_type_);
  ROS_INFO_STREAM("Size of the empty cv::Mat = " << objects_extracted_mat_.size);
  
  // store the sub mat of the objects in a std::vector
  cv::Mat objects_sub_mat;
  // select the object part from the original depth image
  cv::Mat object_sub_mat = depth_matrix_(cv_bounding_box_).clone();
  ROS_INFO_STREAM("size of current sub mat: " << object_sub_mat.size);
  // cv::imwrite("/home/franka/ws_perception/src/vision/doc/pics/test/sub_mat.jpg", object_sub_mat);
  // ROS_INFO_STREAM("Number of sub mats: " << objects_sub_mat.size());

  // replace cooresponding part in the newly created mat with object_sub_mat
  depth_matrix_(cv_bounding_box_).copyTo(objects_extracted_mat_(cv_bounding_box_));
  cv::imwrite("/home/franka/ws_perception/src/vision/doc/pics/test/object_extracted_mat.jpg", objects_extracted_mat_);
  ROS_INFO_STREAM("");
}

void ObjectDepthGeneration::fromMatToMsg()
{
  // store the final mat in a cv_bridge::CvImage object for future convertion
  cv_bridge::CvImage object_extracted_image;
  object_extracted_image.header.frame_id = depth_image_header_.frame_id;
  object_extracted_image.encoding = depth_image_encoding_;
  object_extracted_image.image = objects_extracted_mat_;

  // publish the msg
  // if (has_command_)
  // {
    object_depth_pub_.publish(object_extracted_image.toImageMsg());
    has_command_ = false;
  // }
}

void ObjectDepthGeneration::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  boxes_ = *msg;
  // convert bounding boxes to cv rectangle

  has_boxes_ = true;
}

void ObjectDepthGeneration::depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // save the depth image as a matrix
  depth_image_header_ = msg->header;

  // ROS_INFO_STREAM(depth_image_header_);

  depth_image_ptr_ = cv_bridge::toCvCopy(msg);
  depth_matrix_ = depth_image_ptr_->image;

  depth_matrix_size_.resize(2);
  depth_matrix_size_ = {depth_matrix_.rows, depth_matrix_.cols};

  depth_image_encoding_ = msg->encoding;

  ROS_INFO_STREAM("mat size = " << depth_matrix_size_[1]);

  cv_depth_type_ = cv_type_map_[depth_image_encoding_];

  // ROS_INFO_STREAM("depth encoding format: " << depth_encoding);

  has_image_ = true;
}

void ObjectDepthGeneration::targetNameCallback(const std_msgs::StringConstPtr &msg)
{
  target_name_ = msg->data;
  has_command_ = true;
}