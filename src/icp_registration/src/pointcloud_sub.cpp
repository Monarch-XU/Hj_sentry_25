#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include "ros/ros.h"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

PointCloudXYZI::Ptr cloud_in_; // cloud_in_是当下雷达扫描到的点云

// 全局变量
std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointCloudQueue_;
size_t frameCount_;
pcl::PointCloud<pcl::PointXYZI>::Ptr fusedCloud_ = nullptr;
bool visualizerRunning_ = false; // 用于控制可视化窗口的运行状态
int frame_n;

std::vector<double> rotating_pointcloud_; 
Eigen::Matrix4d transformation_matrix;

PointCloudXYZI::Ptr rotatingPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    pcl::transformPointCloud (*cloud, *out, transformation_matrix);
    return out;
}

// 可视化回调函数（实际上，这不应该是一个回调函数，而是ROS节点的一部分）
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rt_(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_rt_ = rotatingPointcloud(cloud);
    
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(
        cloud, 200, 200, 200);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_rt_color(
        cloud_rt_, 20, 255, 20);
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZI>(cloud,cloud_color, "cloud");
    viewer.addPointCloud<pcl::PointXYZI>(cloud_rt_,cloud_rt_color, "cloud_rt_");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"); // 设置点的大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud_rt_"); // 设置点的大小


    // 创建一个简单的循环来保持窗口打开
    visualizerRunning_ = true;
    while (!viewer.wasStopped ()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 可以在这里添加退出条件，比如检测一个全局变量或ROS消息
    }

    // 清理
    viewer.close();
    
}

// 回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    size_t point_sz = msg->height * msg->width;
    ROS_INFO("point msg size: %lu",point_sz);
    pcl::fromROSMsg(*msg, *cloud_in_);

    // // 将当前帧添加到队列中
    pointCloudQueue_.push_back(cloud_in_);
    frameCount_++;

    // 检查是否达到了三帧
    if (frameCount_ == frame_n) {
        // 融合前三帧的点云（这里仍然只是简单示例）
        fusedCloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& cloud : pointCloudQueue_) {
            *fusedCloud_ += *cloud;
        }

        // 打印融合后的点云大小
        ROS_INFO("Fused PointCloud with %lu points.", fusedCloud_->points.size());

        // 显示融合后的点云（注意：这通常不应该在回调中直接做）
        // 在这里，我为了简化示例而直接调用它，但你应该避免这种做法
        visualizePointCloud(fusedCloud_);

        // 重置队列和帧计数
        pointCloudQueue_.clear();
        frameCount_ = 0;

        // 重置可视化运行状态（在这个简单示例中，我们实际上没有正确关闭它）
        // 在实际应用中，你需要在某个时刻正确设置visualizerRunning_为false来关闭窗口
        // 例如，可以通过ROS消息、全局变量或定时器来实现
        // 在这里，我们简单地重置它，但这会导致如果再次收到三帧点云，则无法再次显示（因为visualizerRunning_仍然是false）
        // 为了这个示例能够多次运行，我们暂时不这样做，但请注意这是不正确的做法
        // visualizerRunning_ = false; // 不要在这里重置！
    }
}



// ROS节点的main函数（示例）
int main(int argc, char **argv) {

    ros::init(argc, argv, "pointcloud_sub");
    ros::NodeHandle nh;

    nh.param<int>("fusion_pointcloud_frames_num", frame_n, 1);
    frameCount_ = 0;
    nh.param<std::vector<double>>("rotating_pointcloud", rotating_pointcloud_, {1, 0, 0, 0,
                                                                                0,-1, 0, 0,
                                                                                0, 0, 0, 0,
                                                                                0, 0, 0, 1});

    transformation_matrix = Eigen::Matrix4d::Identity ();
    transformation_matrix<< 1, 0, 0, 0,
                            0,-1, 0, 0,
                            0, 0,-1, 0,
                            0, 0, 0, 1;
    // int n = 0;
    // for (size_t i = 0; i < 4; ++i){
    //     for (size_t j = 0; j < 4; ++j){
    //         transformation_matrix(i, j) = rotating_pointcloud_[n];
    //         n++;
    //     }
    // }

    ROS_INFO("fusion_pointcloud_frames_num: %d",frame_n);

    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);


    // 订阅点云话题
    ros::Subscriber sub = nh.subscribe("/livox/points", 10, pointCloudCallback);

    // 进入ROS节点的spin循环
    ros::spin();

    // 注意：在这个示例中，我们没有正确关闭可视化窗口，也没有处理visualizerRunning_的重置。
    // 在实际应用中，你需要设计一种机制来优雅地关闭窗口并重置状态。

    return 0;
}