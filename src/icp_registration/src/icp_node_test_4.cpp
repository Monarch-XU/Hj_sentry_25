#include "icp_node_test4.h"

/**
 * 功能：加载参数
 * 传入：
 * 传出：
 */
void loadParams(){
    nh.param<int>("SINGLE_RUN", SINGLE_RUN, 0);
    nh.param<double>("rough_leaf_size", rough_leaf_size, 0.4);
    nh.param<double>("refine_leaf_size", refine_leaf_size, 0.1);
    nh.param<std::string>("pcd_path", pcd_path_, "/home/hj/sentry_ros_25/PCD/205lib.pcd");
    nh.param<int>("rough_iter_", rough_iter_, 1);
    nh.param<int>("refine_iter_", refine_iter_, 1);
    nh.param<std::string>("map_frame_id", map_frame_id_, "map");
    nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    nh.param<std::string>("range_odom_frame_id", range_odom_frame_id_, "range_odom");
    nh.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
    nh.param<double>("thresh", thresh_,  0.15);
    nh.param<double>("xy_offset", xy_offset_,  0.2);
    nh.param<double>("yaw_offset", yaw_offset_,  30.0);
    yaw_offset_ *= M_PI / 180.0;
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/points");
    nh.param<std::string>("initialpose_topic", initialpose_topic, "/initialpose_pub");
    nh.param<std::vector<double>>("initial_pose", initial_pose_vec_,  {0, 0, 0, 0, 0, 0});

    nh.param<std::vector<double>>("rotating_pointcloud", rotating_pointcloud_, {1, 0, 0, 0,
                                                                                0, 1, 0, 0,
                                                                                0, 0, 1, 0,
                                                                                0, 0, 0, 1});

}