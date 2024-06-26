#ifndef OBJECTDEPTHGENERATION
#define OBJECTDEPTHGENERATION

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

#include <map>

class ObjectDepthGeneration
{
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef PointCloudT::Ptr CloudPtrT;

    typedef pcl::PointXYZL PointTL;
    typedef pcl::PointCloud<PointTL> PointCloudL;
    typedef PointCloudL::Ptr CloudPtrL;

  public:
    bool init();        // initialize the subs and pubs
    bool update();      // call the process function and publish msgs

  private: // topics
    std::string camera_info_topic;
    std::string bounding_box_topic;
    std::string object_cloud_topic;
    std::string depth_image_topic;

    std::string image_save_path_;

  private: // node handle, subs and pubs
    ros::NodeHandle nh_;

    ros::Subscriber camera_info_sub_;
    ros::Subscriber bounding_boxes_sub_;
    ros::Subscriber object_cloud_sub_;
    ros::Subscriber depth_image_sub_;
    ros::Subscriber target_name_sub_;

    ros::Publisher image_boxes_pub_;
    ros::Publisher object_depth_pub_;

  private: // variables to be initialized in init function
    std::map<std::string, int> cv_type_map_;
  
  private: // callback functions
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg);
    void objectCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void targetNameCallback(const std_msgs::StringConstPtr &msg);
  
  private:
    // For boundingBoxesCallback
    std::vector<darknet_ros_msgs::BoundingBox> boxes_;
    cv::Rect cv_bounding_box_;
    int boxes_num_;
    bool has_boxes_;
    std::vector<std::string> detected_objects_;

    // For depthImageCallback 
    std_msgs::Header depth_image_header_;
    cv::Mat depth_matrix_;
    std::vector<int> depth_matrix_size_;
    std::string depth_image_encoding_;
    int cv_depth_type_;
    bool has_image_;

    // For commandCallback
    std::string target_name_;
    bool has_command_;

  private:
    void objectDepthExtraction();
    void fromMatToMsg();
    void msgIntergration();
    int boxSelection();

  private:
    // for objectDepthExtraction
    cv::Mat objects_extracted_mat_;

    // for fromMatToMsg
    sensor_msgs::Image objects_depth_image_;
};

#endif