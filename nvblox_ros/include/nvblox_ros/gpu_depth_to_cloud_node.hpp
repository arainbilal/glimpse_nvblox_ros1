#ifndef GPU_DEPTH_TO_CLOUD_NODE_HPP_
#define GPU_DEPTH_TO_CLOUD_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <nvblox_ros/conversions.hpp>

/// PCL
//#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

/// CUDA
//#include <cuda_runtime.h>

//constexpr cudaMemoryType kDefaultImageMemoryType = cudaMemoryType::kDevice;

class GpuCloudNode
{
public:

  GpuCloudNode(ros::NodeHandle& nodeHandle);
  void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_img_ptr);

//  bool depthToCloudPCLConversion(const sensor_msgs::ImageConstPtr& depth_img_ptr,
//                                 const image_geometry::PinholeCameraModel& model,
//                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_msg);
//  void populateFromBuffer(int rows, int cols, const ElementType* buffer,
//                            cudaMemoryType memory_type = kDefaultImageMemoryType);

private:
  ros::Subscriber depth_sub1_;
  ros::Subscriber depth_camera_info_sub1_;
  ros::Publisher pub_point_cloud_;
  ros::NodeHandle nodeHandle_;
  bool init_camera_info_;
  sensor_msgs::CameraInfo last_cam_info_;

  /// Caches for GPU images
  nvblox::DepthImage depth_image_;
  nvblox::RosConverter converter_;
};     /// GpuCloudNode
#endif /// GPU_DEPTH_TO_CLOUD_NODE_HPP_
