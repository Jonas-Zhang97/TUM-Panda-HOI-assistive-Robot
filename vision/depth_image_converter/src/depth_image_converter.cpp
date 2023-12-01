#include <depth_image_converter/depth_image_converter.h>

bool DepthImageConverter::init()
{
  std::string camera_info_topic;
  nh_.getParam("camera_info_topic", camera_info_topic);

  // Initialize subscribers
  point_cloud_sub_ = nh_.subscribe("/hoi/objects_cloud", 1, &DepthImageConverter::pointCloudCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &DepthImageConverter::cameraInfoCallback, this);
  // command_sub_ = nh_.subscribe("/hoi/convert_cloud_to_image", 1, &DepthImageConverter::commandCallback, this);

  // Initialize publishers
  depth_image_pub_ = nh_.advertise<sensor_msgs::Image>("/hoi/image_from_cloud", 1);

  objects_cloud_.reset(new PointCloud);
  objects_cloud_cam_.reset(new PointCloud);
  objects_cloud_color_.reset(new PointCloudC);

  return true;
}

void DepthImageConverter::update()
{
  if (has_cloud_ && has_K_)
  {
    cloudTransformation();
    imageGenerator();

    has_cloud_ = false;
    has_K_ = false;
  }
}

int DepthImageConverter::cloudTransformation()
{
  // ROS_INFO_STREAM("number of points in objects_cloud_ = " << objects_cloud_->size());    // Here is no point
  // Get transformation
  tf::TransformListener listener;
  tf::StampedTransform transform_stamped;
  try
  {
    listener.waitForTransform("camera_color_optical_frame", objects_cloud_header_frame_, ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("camera_color_optical_frame", objects_cloud_header_frame_, ros::Time::now(), transform_stamped);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    // ros::Duration(1.0).sleep();
    return -1;
  }

  Eigen::Affine3d T_cam_base;
  // tf::transformTFToEigen (const tf::Transform &t, Eigen::Affine3d &e)
  tf::transformTFToEigen(transform_stamped, T_cam_base);

  pcl::transformPointCloud(*objects_cloud_, *objects_cloud_cam_, T_cam_base);

  // ROS_INFO_STREAM("number of points in objects_cloud_cam_ = " << objects_cloud_cam_->size());    // there is no points here!
  
  return 1;
}

void DepthImageConverter::imageGenerator()
{
  cv::Mat depth_mat;
  depth_mat.create(image_size_[0], image_size_[1], CV_16UC1);
  // ROS_INFO_STREAM("Size of the empty mat = " << depth_mat.size);
  depth_mat = 0;

  double max_x = 0;
  double min_x = 1280;

  for (size_t i = 0; i < objects_cloud_cam_->size(); ++i)
  {
    // Get the 3D position of the i-th point w.r.t. camera_color_optical_frame
    Eigen::Vector3d curr_point_3d(objects_cloud_cam_->at(i).x, 
                                  objects_cloud_cam_->at(i).y, 
                                  objects_cloud_cam_->at(i).z);
    // ROS_INFO_STREAM("curr_point_3d = " << curr_point_3d.transpose());
    // ros::Duration(1.0).sleep();
    
    // Project the point into the pixel coordinate with homogene coordinate
    Eigen::Vector3d curr_point_pixel_hom;
    curr_point_pixel_hom = K_*curr_point_3d;
    // ROS_INFO_STREAM("curr_point_pixel_hom = " << curr_point_pixel_hom.transpose());
    
    // Get the 2D position of the point in pixel coordinate, in round value
    Eigen::Vector2f curr_point_pixel(curr_point_pixel_hom[0] / curr_point_pixel_hom[2], 
                                     curr_point_pixel_hom[1] / curr_point_pixel_hom[2]);
    // ROS_INFO_STREAM("curr_point_pixel = " << curr_point_pixel.transpose());

    // Justify, if this point is inside the range of depth image
    if (curr_point_pixel[1] /* y */ <= image_size_[0] /* rows */ && 
        curr_point_pixel[0] /* x */ <= image_size_[1] /* cols */ &&
        curr_point_pixel[1] >= 0 &&
        curr_point_pixel[0] >= 0)
    {
      depth_mat.at<ushort>(curr_point_pixel[1], curr_point_pixel[0]) = std::round(objects_cloud_cam_->at(i).z * std::pow(10, 3));
    }
    else
    {
      ROS_WARN_STREAM("trying to map a point to depth image but current point is out of the image range");
      // If out, skip this point
      continue;
    }
  }

  cv::imwrite("/home/franka/ws_perception/src/vision/doc/pics/test/depth_mat.jpg", depth_mat);

  cv_bridge::CvImage depth_cv_image;
  depth_cv_image.header.frame_id = "camera_color_optical_frame";
  depth_cv_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_cv_image.image = depth_mat;

  ROS_INFO_STREAM("publishing depth image to /hoi/image_from_cloud");
  depth_image_pub_.publish(depth_cv_image.toImageMsg());
}

void DepthImageConverter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, *objects_cloud_color_);
  
  pcl::copyPointCloud(*objects_cloud_color_, *objects_cloud_);

  // ROS_INFO_STREAM("number of points in objects_cloud_ = " << objects_cloud_->size());

  objects_cloud_header_frame_ = msg->header.frame_id;
  // ROS_INFO_STREAM(objects_cloud_header_frame_);

  has_cloud_ = true;
}

void DepthImageConverter::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  for(size_t i = 0; i < 9; ++i)
  {
    K(i) = msg->K[i];
  }
  K_ = K.transpose();

  // ROS_INFO_STREAM("K = " << K_);
  
  // image_size[0]: how many rows; image_size_[1]: how many columns
  image_size_.resize(2);
  image_size_ = {msg->height, msg->width};

  has_K_ = true;
}
