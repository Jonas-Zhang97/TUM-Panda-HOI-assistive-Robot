void PlaneSegmentation::generateDepthImage()
{
  // convert PointCloudXYZRGB to PointCloudXYZ
  CloudPtrG objects_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*objects_cloud_, *objects_xyz);

  // get the transformation from base link to camera link
  tf::TransformListener listener;
  tf::StampedTransform T_cam_base;

  try
  {
    listener.waitForTransform("camera_depth_optical_frame", objects_xyz->header.frame_id, ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("camera_depth_optical_frame", objects_xyz->header.frame_id, ros::Time::now(), T_cam_base);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // convert a tf transform into Eigen
  Eigen::Affine3d eigen_transform;
  tf::transformTFToEigen(T_cam_base, eigen_transform);

  // create a empty depth image
  cv::Mat depth_mat(720, 1280, CV_16UC1, cv::Scalar(0));;

  // ROS_INFO_STREAM("depth image size: " << depth_mat.size);
  // ROS_INFO_STREAM("number of points = " << objects_xyz->size());

  for (int i = 0; i < objects_xyz->size(); ++i)
  {
    // get the current point in base frame
    pcl::PointXYZ curr_pointxyz = objects_xyz->at(i);
    Eigen::Vector3d curr_point(curr_pointxyz.x, curr_pointxyz.y, curr_pointxyz.z);
    
    // transform the point in camera frame using eigen_transform
    Eigen::Vector3d curr_point_cam = eigen_transform * curr_point;
    
    // transform the point in pixel frame by multiplying it with K_
    Eigen::Vector3d curr_point_pixel_3d;
    curr_point_pixel_3d = K_ * curr_point_cam;
    Eigen::Vector2d curr_point_pixel(curr_point_pixel_3d[0] / curr_point_pixel_3d[2], curr_point_pixel_3d[1] / curr_point_pixel_3d[2]);

    ROS_INFO_STREAM("pixel coordinates of current point: " << curr_point_pixel.transpose());
    
    depth_mat.at<double>(curr_point_pixel[1], curr_point_pixel[0]) = std::ceil(1000 * curr_point_cam[2]);

    ROS_INFO_STREAM("depth value = " << curr_point_cam[2]);
  }

  depth_image_.header.frame_id = "camera_color_optical_frame";
  depth_image_.encoding = "16UC1";
  depth_image_.image = depth_mat;
}