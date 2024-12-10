// std
#include <filesystem>
#include <mutex>
#include <thread>
#include <functional>
#include <string>

// ROS1
#include "ros/ros.h"
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

// TF_ROS1
#include <tf/transform_broadcaster.h>  
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl_ros/point_cloud.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h> // 如果你需要从PCD文件加载点云
#include <pcl/io/ply_io.h>
#include <pcl/console/time.h>   // TicToc

// Eigen
#include<Eigen/Dense>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sensor_msgs/PointCloud2.h>
#include <boost/shared_ptr.hpp>

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

std::string pointcloud_topic_;
ros::Subscriber pointcloud_sub_;  
ros::Subscriber initial_pose_sub_;
std::mutex mutex_;
std::unique_ptr<std::thread> tf_publisher_thread_;
pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;
int rough_iter_;
int refine_iter_;

pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

PointCloudXYZI::Ptr cloud_; // 原始地图点云
PointCloudXYZI::Ptr cloud_in_; // cloud_in_是当下雷达扫描到的点云
PointCloudXYZIN::Ptr rough_map_; // 原始地图点云粗处理点云
PointCloudXYZIN::Ptr refine_map_; // 原始地图点云精处理点云

PointCloudXYZIN::Ptr align_point; // 配准后的点云

geometry_msgs::TransformStamped map_to_odom_;
tf::Transform map_to_odom_transform;
std::string pcd_path_;
std::string map_frame_id_, odom_frame_id_, range_odom_frame_id_, laser_frame_id_;
bool success_;
double rough_leaf_size_, refine_leaf_size_;
double score_;
double thresh_; // 匹配分数阈值
double xy_offset_;
double yaw_offset_;
double yaw_resolution_;
std::vector<double> initial_pose_vec_;
geometry_msgs::Pose initial_pose_;
bool is_ready_;
bool pointcloud_ready;

bool first_scan_;
int SINGLE_RUN; // 单次运行调试标识符

void loadParams();