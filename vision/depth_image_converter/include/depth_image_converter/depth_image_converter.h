#ifndef DEPTH_IMAGE_CONVERTER
#define DEPTH_IMAGE_CONVERTER

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class DepthImageConverter
{
  public:  // Define types for convenience
    typedef pcl::PointXYZRGB PointTC;               // The Point Type
    typedef pcl::PointCloud<PointTC> PointCloudC;   // The PointCloud Type
    typedef PointCloudC::Ptr CCloudPtr;             // The PointCloud Pointer Type

    typedef pcl::PointXYZ PointT;                   // The Point Type
    typedef pcl::PointCloud<PointT> PointCloud;     // The PointCloud Type
    typedef PointCloud::Ptr CloudPtr;               // The PointCloud Pointer Type

  public:  // Public functions to be called in the executable
    bool init();
    void update();

  private:  // For ROS subscribers and publishers
    ros::NodeHandle nh_;
    
    ros::Subscriber point_cloud_sub_;            // Subscribe to the /hoi/objects_cloud
    ros::Subscriber camera_info_sub_;
    ros::Subscriber command_sub_;

    ros::Publisher depth_image_pub_;
    ros::Publisher cloud_cam_pub_;

  private:  // Callbacks for subscribers
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void commandCallback(const std_msgs::BoolConstPtr &msg);

  private:  // Local variables to save the subscribed messages
    // For pointCloudCallback
    CCloudPtr objects_cloud_color_;
    std::string objects_cloud_header_frame_;
    CloudPtr objects_cloud_;
    bool has_cloud_ = false;
    // For cameraInfoCallback
    Eigen::Matrix3d K_;
    std::vector<int> image_size_;
    bool has_K_ = false;
    // For commandCallback
    bool command_ = false;

  private:  // Main functionalities
    int cloudTransformation();  // Transform the point cloud to camera_color_optical_frame
    void imageGenerator();    // Convert the point cloud to 2D pixel frame and generate depth image

  private:  // Local variables for member functions
    // For cloudTransformation
    CloudPtr objects_cloud_cam_;
    // For imageGenerator
    sensor_msgs::Image depth_image_;
    
};

#endif