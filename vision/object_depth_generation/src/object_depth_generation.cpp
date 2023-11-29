#include <object_depth_generation/object_depth_generation.h>

bool ObjectDepthGeneration::init()
{
  bounding_box_topic = "/darknet_ros/bounding_boxes";

  std::string input_image_topic;

  nh_.getParam("/object_depth_generation_node/topic", input_image_topic);
  nh_.getParam("image_path", image_save_path_);

  // ROS_INFO_STREAM(image_save_path_);

  if (input_image_topic == "/camera/depth/image_rect_raw")
  {
    ROS_ERROR_STREAM("please use depth image that is aligned to the color image");
    return false;
  }

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

bool ObjectDepthGeneration::update()
{
  if (has_boxes_ && has_image_ && has_command_)
  {
    if (boxSelection() == -1)
    {
      ROS_WARN_STREAM("No object is detected");
      return false;
    }
    objectDepthExtraction();
    fromMatToMsg();
    
    // reset the flags
    has_boxes_ = false;
    has_image_ = false;
    has_command_ = false;
  }
  return true;
}

int ObjectDepthGeneration::boxSelection()
{
  if (boxes_.size() == 0)
  {
    return -1;
  }
  for (int i = 0; i < boxes_.size(); ++i)
  {
    if (boxes_[i].Class == target_name_)
    {
      cv_bounding_box_.x = boxes_[i].xmin;
      cv_bounding_box_.y = boxes_[i].ymin;
      cv_bounding_box_.width = boxes_[i].xmax-boxes_[i].xmin;
      cv_bounding_box_.height = boxes_[i].ymax-boxes_[i].ymin;
      ROS_INFO_STREAM("cv bounding box: " << cv_bounding_box_);
    }
  }

  return 1;
}

void ObjectDepthGeneration::objectDepthExtraction()
{
  objects_extracted_mat_.release();
  // create an empty mat
  objects_extracted_mat_.create(depth_matrix_size_[0], depth_matrix_size_[1], cv_depth_type_);
  objects_extracted_mat_ = 0;

  ROS_INFO_STREAM("Size of the empty cv::Mat = " << objects_extracted_mat_.size);
  std::string empty_mat_path;
  empty_mat_path = image_save_path_;
  empty_mat_path = empty_mat_path.append("empty_mat.jpg");
  cv::imwrite(empty_mat_path, objects_extracted_mat_);
  
  depth_matrix_(cv_bounding_box_).copyTo(objects_extracted_mat_(cv_bounding_box_));
  std::string depth_mat_path;
  depth_mat_path = image_save_path_;
  depth_mat_path = image_save_path_.append("object_extracted_mat.jpg");
  cv::imwrite(depth_mat_path, objects_extracted_mat_);
  ROS_INFO_STREAM("Image saved at /vision/doc/pics/test/object_extracted_mat.jpg");
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
  boxes_.clear();
  for (const darknet_ros_msgs::BoundingBox& b_box: msg->bounding_boxes)
  {
    boxes_.push_back(b_box);
  }
  // convert bounding boxes to cv rectangle

  has_boxes_ = true;
}

void ObjectDepthGeneration::depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  depth_matrix_.release();
  // save the depth image as a matrix
  depth_image_header_ = msg->header;

  // ROS_INFO_STREAM(depth_image_header_);

  cv_bridge::CvImagePtr depth_image_ptr;
  depth_image_ptr = cv_bridge::toCvCopy(msg);
  depth_matrix_ = depth_image_ptr->image;

  depth_matrix_size_.resize(2);
  depth_matrix_size_ = {depth_matrix_.rows, depth_matrix_.cols};

  depth_image_encoding_ = msg->encoding;

  // ROS_INFO_STREAM("mat size = " << depth_matrix_size_[1]);

  cv_depth_type_ = cv_type_map_[depth_image_encoding_];

  // ROS_INFO_STREAM("depth encoding format: " << depth_encoding);

  has_image_ = true;
}

void ObjectDepthGeneration::targetNameCallback(const std_msgs::StringConstPtr &msg)
{
  target_name_ = msg->data;
  has_command_ = true;
}