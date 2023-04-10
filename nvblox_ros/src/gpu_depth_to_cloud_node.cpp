#include <nvblox_ros/gpu_depth_to_cloud_node.hpp>
/// Nvblox backend to perform conversions using CUDA
#include <nvblox/core/cuda/warmup.h>
#include <nvblox_ros/conversions.hpp>
//#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>

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

  tic = ros::Time::now();
  int count = camera.height()* camera.width();
  std::vector<float3> point(count, { 0,0,0 });
  converter_.pointcloudVectorFromDepth(depth_image_, camera, (float*)point.data());
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("2. Depth to poincloud conversion took "<< elapsed << " sec");

#if(1)
  /// This conversion directly copy the points to the sensor_msgs
  tic = ros::Time::now();
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_img_ptr->header;
  cloud_msg->height = depth_img_ptr->height;
  cloud_msg->width  = depth_img_ptr->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  for(unsigned int i=0; i<count; ++i,++iter_x, ++iter_y, ++iter_z)
  {
      *iter_x = point[i].x;
      *iter_y = point[i].y;
      *iter_z = point[i].z;
  }
  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("3. Pointcloud2 sensor_msg conversion took "<< elapsed << " sec");

  if (pub_point_cloud_.getNumSubscribers() > 0u)
  {
    pub_point_cloud_.publish (cloud_msg);
  }
#endif

#if(0)
  tic = ros::Time::now();
  /// Fill in the pcl pointcloud msg and then convert using pcl_ros to sensor_msgs.
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  tmp->width = 0;
  tmp->height = 1;
  tmp->is_dense = false;
  tmp->points.resize(tmp->width * tmp->height);
  tmp->header.frame_id = depth_img_ptr->header.frame_id;

  for (int i = 0; i<count; i++)
  {
      pcl::PointXYZ pt;
      pt.x = point[i].x;
      pt.y = point[i].y;
      pt.z = point[i].z;
      tmp->points.push_back(pt);
      tmp->width++;
  }
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_img_ptr->header;
  pcl::toROSMsg(*tmp, *cloud_msg);

  toc = ros::Time::now();
  elapsed = toc.toSec() - tic.toSec();
  ROS_INFO_STREAM("3. Pointcloud2 sensor_msg conversion took "<< elapsed << " sec");

  if (pub_point_cloud_.getNumSubscribers() > 0u)
  {
    pub_point_cloud_.publish (cloud_msg);
  }

#endif


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


