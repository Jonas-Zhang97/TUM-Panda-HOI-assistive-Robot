#include <object_depth_generation/object_depth_generation.h>

bool ObjectDepthGeneration::init()
{
  bounding_box_topic = "/darknet_ros/bounding_boxes";
  depth_image_topic = "/camera/aligned_depth_to_color/image_raw";

  bounding_boxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>(bounding_box_topic, 1, &ObjectDepthGeneration::boundingBoxesCallback, this);
  depth_image_sub_ = nh_.subscribe<sensor_msgs::Image>(depth_image_topic, 1, &ObjectDepthGeneration::depthImageCallback, this);

  object_depth_pub_ = nh_.advertise<sensor_msgs::Image>("/object_depth_image", 1);

  cv_type_map_["16UC1"] = CV_16UC1;

  has_boxes_ = false;
  has_image_ = false;
  return true;
}

void ObjectDepthGeneration::update()
{
  if (has_boxes_ && has_image_)
  {
    objectDepthExtraction();
    fromMatToMsg();
  }

  // reset the flags
  has_boxes_ = false;
  has_image_ = false;
}

void ObjectDepthGeneration::objectDepthExtraction()
{
  // create an empty mat
  objects_extracted_mat_.create(depth_matrix_size_[0], depth_matrix_size_[1], cv_depth_type_);
  // ROS_INFO_STREAM("Size of the empty cv::Mat = " << objects_extracted_mat_.size);
  
  // store the sub mat of the objects in a std::vector
  std::vector<cv::Mat> objects_sub_mat;
  // select the object part from the original depth image
  for (int i = 0; i < boxes_num_; ++i)
  {
    cv::Mat object_sub_mat = depth_matrix_(cv_bounding_boxes_[i]).clone();
    // ROS_INFO_STREAM("size of current sub mat: " << object_sub_mat.size);
    objects_sub_mat.push_back(object_sub_mat);
  }
  // ROS_INFO_STREAM("Number of sub mats: " << objects_sub_mat.size());

  // replace cooresponding part in the newly created mat with object_sub_mat
  for (int i = 0; i < boxes_num_; ++i)
  {
    objects_sub_mat[i].copyTo(objects_extracted_mat_(cv_bounding_boxes_[i]));
  }
}

void ObjectDepthGeneration::fromMatToMsg()
{
  // store the final mat in a cv_bridge::CvImage object for future convertion
  cv_bridge::CvImage object_extracted_image;
  object_extracted_image.header.frame_id = depth_image_header_.frame_id;
  object_extracted_image.encoding = depth_image_encoding_;
  object_extracted_image.image = objects_extracted_mat_;

  // publish the msg
  object_depth_pub_.publish(object_extracted_image.toImageMsg());
}

void ObjectDepthGeneration::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
  // convert bounding boxes to cv rectangle
  cv_bounding_boxes_.clear();
  for (const darknet_ros_msgs::BoundingBox& b_box : msg->bounding_boxes)
  {
    cv::Rect cv_bounding_box(b_box.xmin, b_box.ymin, b_box.xmax-b_box.xmin+1, b_box.ymax-b_box.ymin+1);
    // ROS_INFO_STREAM("cv bounding box: " << cv_bounding_box);
    cv_bounding_boxes_.push_back(cv_bounding_box);
  }
  boxes_num_ = cv_bounding_boxes_.size();

  has_boxes_ = true;
}

void ObjectDepthGeneration::depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // save the depth image as a matrix
  depth_image_header_ = msg->header;
  depth_image_ptr_ = cv_bridge::toCvCopy(msg);
  depth_matrix_ = depth_image_ptr_->image;

  depth_matrix_size_.resize(2);
  depth_matrix_size_ = {depth_matrix_.rows, depth_matrix_.cols};

  depth_image_encoding_ = msg->encoding;

  // ROS_INFO_STREAM("mat size = " << depth_matrix_size_[1]);

  cv_depth_type_ = cv_type_map_[depth_image_encoding_];

  // ROS_INFO_STREAM("depth encoding format: " << depth_encoding);

  has_image_ = true;
}