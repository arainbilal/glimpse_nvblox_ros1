#include <nvblox_ros/gpu_depth_to_cloud_node.hpp>
/// Nvblox backend to perform conversions using CUDA
#include <nvblox/core/cuda/warmup.h>
#include <nvblox_ros/conversions.hpp>

GpuCloudNode::GpuCloudNode(ros::NodeHandle& nodeHandle)
  : nodeHandle_(nodeHandle)
  , init_camera_info_(false)
{
  // Queue size is important, since we have to wait synced messages.
  constexpr int kQueueSize = 20;

  std::string depth_image_1_topic = "/camera/depth/image_raw";
  std::string camera_info_1_topic = "/camera/depth/camera_info";

  depth_sub1_ = nodeHandle_.subscribe(depth_image_1_topic, 20, &GpuCloudNode::depthImageCallback, this);
  depth_camera_info_sub1_ = nodeHandle_.subscribe(camera_info_1_topic, 20, &GpuCloudNode::depthInfoCallback, this);
  pub_point_cloud_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/cloud_node/points",1,false);
}

void GpuCloudNode::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
{
  /// ROS_INFO_STREAM("Received depth info  " << camera_info_msg->header.frame_id);
  last_cam_info_ = *camera_info_msg;
  if(!init_camera_info_)
  {
    init_camera_info_=true;
  }
}

void GpuCloudNode::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_img_ptr)
{
  /// ROS_INFO_STREAM("Received depth image  " << depth_img_ptr->encoding);
  if(!init_camera_info_)
  {
    return;
  }

  /// Convert camera info message to nvblox camera object.
  ros::Time tic, toc;
  double elapsed;
  /// I need to implement the camera info.
  tic = ros::Time::now();
  nvblox::Camera camera = converter_.cameraFromMessage(last_cam_info_);
  // Convert the depth image.
  if (!converter_.depthImageFromImageMessage(depth_img_ptr, &depth_image_))
  {
    ROS_ERROR("Failed to transform depth image.");
  }
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("1. Depth image to GPU cache conversion took "<< elapsed << " sec");

  /// Access the depth in the GPU

  sensor_msgs::PointCloud2* out_cloud;
  converter_.pointcloudFromDepth(depth_image_, camera, out_cloud);



#if(0)
  /// 1. Conversion from image msg to PCL Pointcloud
  ros::Time tic, toc;
  double elapsed;
  tic = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
  depthToCloudPCLConversion(depth_img_ptr, model_, pcl_msg);
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("1. Depth image to PCL conversion took "<< elapsed << " sec");

  /// 2. Filter the pointcloud
  tic = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr filter_msg(new pcl::PointCloud<pcl::PointXYZ>);
  filterCloud(pcl_msg, filter_msg);
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("2. Voxel filter took "<< elapsed << " sec");

  /// 3. Conversion PCL Pointcloud to sensor_msgs::Pointcloud
  tic = ros::Time::now();
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_img_ptr->header;
  pcl::toROSMsg(*filter_msg, *cloud_msg);
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("3. PCL to sensor_msg conversion took "<< elapsed << " sec");


  if (pub_point_cloud_.getNumSubscribers() > 0u)
  {
    pub_point_cloud_.publish (cloud_msg);
  }
#endif
}

#if(0)
  /// Helper functions
  void depthToCloudDirectConversion(const sensor_msgs::ImageConstPtr& depth_img_ptr,
                                    const image_geometry::PinholeCameraModel& model,
                                    sensor_msgs::PointCloud2::Ptr& cloud_msg);

  void depthToCloudPCLConversion(const sensor_msgs::ImageConstPtr& depth_img_ptr,
                                 const image_geometry::PinholeCameraModel& model,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_msg);

  void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& out_cloud);

  static inline bool valid_16uc1(uint16_t depth) { return depth != 0; }
  static inline float toMeters_16uc1(uint16_t depth) { return depth * 0.001f; } // originally mm

  static inline bool valid_32fc1(float depth) { return std::isfinite(depth); }
  static inline float toMeters_32fc1(float depth) { return depth; }
#endif


int main(int argc, char * argv[])
{
  ros::init(argc, argv, "gpu_depth_to_cloud_node");
  ros::NodeHandle nh("~");
  GpuCloudNode GpuCloudNode(nh);
  ros::spin();
  return 0;
}


