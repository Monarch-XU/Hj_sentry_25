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

PointCloudXYZI::Ptr fused_Cloud; // 多帧融合点云
int frame_count; // 帧数计数器
int frame_num; // 融合帧数

PointCloudXYZIN::Ptr align_point; // 配准后的点云

geometry_msgs::TransformStamped map_to_odom_;
tf::Transform map_to_odom_transform;
std::string pcd_path_;
std::string map_frame_id_, odom_frame_id_, range_odom_frame_id_, laser_frame_id_;
bool success_;
double rough_leaf_size_, refine_leaf_size_;
double score_;
double thresh_;
double xy_offset_;
double yaw_offset_;
double yaw_resolution_;
geometry_msgs::Pose initial_pose_;
bool is_ready_;
bool pointcloud_ready;
bool first_scan_;
int SINGLE_RUN;
bool run_once;
std::vector<double> rotating_pointcloud_; 
Eigen::Matrix4d transformation_matrix_;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
PointCloudXYZI::Ptr rotatingPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
static PointCloudXYZIN::Ptr addNorm(PointCloudXYZI::Ptr cloud);
Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess);
PointCloudXYZI::Ptr rotatingPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
tf::Transform mapMatrixToTransform(const Eigen::Matrix4d& transformation_matrix);
Eigen::Matrix4d matrixCorrection(const Eigen::Matrix4d& transformation_matrix);


// test
void pointCloudCallback_test(const sensor_msgs::PointCloud2ConstPtr& msg);
void showPointCloud(PointCloudXYZI::Ptr cloud, std::string name, int B, int G, int R);
void showPointCloud(PointCloudXYZIN::Ptr cloud, std::string name, int B, int G, int R);
void pclViewer(PointCloudXYZI::Ptr prior_point_cloud, PointCloudXYZI::Ptr pointcloud_before_tf, PointCloudXYZIN::Ptr pointcloud_after_tf);


int main(int argc, char *argv[])  {  
    ros::init(argc,argv,"icp_node_test4_1");  
    ros::NodeHandle nh("~");
    setlocale(LC_ALL,"");

    map_to_odom_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    map_to_odom_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    pointcloud_ready = false;
    first_scan_ = true;
    is_ready_ = false;
    run_once = false;

    nh.param<int>("SINGLE_RUN", SINGLE_RUN, 0);
    nh.param<int>("frame_num", frame_num, 3);


    // 初始化一个指向pcl::PointCloud<pcl::PointXYZI>的智能指针，用于存储输入的点云数据
    cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    //// TODO:改成用服务器读参
    // double rough_leaf_size = (double)fs["rough_leaf_size"];
    // double refine_leaf_size = (double)fs["refine_leaf_size"];
    double rough_leaf_size, refine_leaf_size;
    nh.param<double>("rough_leaf_size", rough_leaf_size, 0.4);
    nh.param<double>("refine_leaf_size", refine_leaf_size, 0.1);

    ROS_INFO("refine_leaf_size: %lf\n",refine_leaf_size);
    // std::cout << "refine_leaf_size: " << refine_leaf_size <<std::endl;

    // 设置粗糙和精细体素滤波器叶大小。
    voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
    voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);

    // 读取PCD文件路径参数
    nh.param<std::string>("pcd_path", pcd_path_, "/home/hj/sentry_ros_25/PCD/205lib.pcd");
   
    // 用PCL的PCDReader类读取PCD文件，并将点云数据存储在cloud指针指向的对象中
    pcl::PCDReader reader;
    // cloud_in_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    fused_Cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); // 初始化融合点云
    frame_count = 0; // 初始化点云融合帧数计数器
    cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); // 原始点云
    reader.read(pcd_path_, *cloud_);

    // showPointCloud(cloud_, "cloud_", 0, 255, 0);

    // 将读取的点云数据作为输入传递给精细体素滤波器，并应用滤波器进行下采样
    voxel_refine_filter_.setInputCloud(cloud_);
    voxel_refine_filter_.filter(*cloud_);

    // Add normal to the pointcloud
    // 调用addNorm函数为精细处理后的点云添加法线信息
    refine_map_ = addNorm(cloud_);
    // 创建粗糙处理点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(new pcl::PointCloud<pcl::PointXYZI>); 
    // 创建滤波后粗糙点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(new pcl::PointCloud<pcl::PointXYZI>);
    // 复制精细处理后的点云到粗糙处理点云(不复制法线)
    pcl::copyPointCloud(*refine_map_, *point_rough);

    // showPointCloud(point_rough, "point_rough", 60, 0, 255);

    nh.param<int>("rough_iter_", rough_iter_, 1);
    nh.param<int>("refine_iter_", refine_iter_, 1);
    /*调用voxel_rough_filter_的filter方法，对point_rough中的点进行下采样处理，并将结果存储在filterd_point_rough中。
    经过这一步，filterd_point_rough中的点数量会减少，点的分布会变得更加稀疏，但整体上仍然保留了原始点云的形状特征。*/
    voxel_rough_filter_.setInputCloud(point_rough);
    voxel_rough_filter_.filter(*filterd_point_rough);
    // 为滤波后的粗糙点云添加法线信息
    rough_map_ = addNorm(filterd_point_rough);
    // 设置粗略ICP/初步配准器的最大迭代次数为rough_iter_
    icp_rough_.setMaximumIterations(rough_iter_);
    // 设置精细ICP/精细配准器的目标点云为rough_map_
    icp_rough_.setInputTarget(rough_map_);
    // 设置精细ICP/精细配准器的最大迭代次数为refine_iter_
    icp_refine_.setMaximumIterations(refine_iter_);
    // 设置精细ICP/精细配准器的目标点云为refine_map_
    icp_refine_.setInputTarget(refine_map_);

    ROS_INFO("rough_iter_:%d",rough_iter_);
    ROS_INFO("refine_iter_:%d",refine_iter_);
    // std::cout << "rough_iter_: " << rough_iter_ <<std::endl;
    // std::cout << "refine_iter_: " << refine_iter_ <<std::endl;


    // 打印点云大小
    ROS_INFO("PCD大小： %ld, %ld", refine_map_->size(), rough_map_->size());

    // Parameters
    //// TODO:改成用服务器读参
    nh.param<std::string>("map_frame_id", map_frame_id_, "map");
    nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    nh.param<std::string>("range_odom_frame_id", range_odom_frame_id_, "range_odom");
    nh.param<std::string>("laser_frame_id", laser_frame_id_, "laser");
    nh.param<double>("thresh", thresh_,  0.15);
    nh.param<double>("xy_offset", xy_offset_,  0.2);
    nh.param<double>("yaw_offset", yaw_offset_,  30.0);
    yaw_offset_ *= M_PI / 180.0;

    std::vector<double> initial_pose_vec; 
    nh.param<std::vector<double>>("initial_pose", initial_pose_vec,  {0, 0, 0, 0, 0, 0});

    nh.param<std::vector<double>>("rotating_pointcloud", rotating_pointcloud_, {1, 0, 0, 0,
                                                                                0, 1, 0, 0,
                                                                                0, 0, 1, 0,
                                                                                0, 0, 0, 1});
    transformation_matrix_ = Eigen::Matrix4d::Identity ();
    int n = 0;
    for (size_t i = 0; i < 4; ++i){
        for (size_t j = 0; j < 4; ++j){
            transformation_matrix_(i, j) = rotating_pointcloud_[n];
            n++;
        }
    }

    try {
    // 设置初始位置三维坐标
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf::Quaternion q; 
    // 通过四元数设置初始位置row、pitch、yaw角度
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4), initial_pose_vec.at(5));
    } catch (const std::out_of_range &ex) {
        // 若initial_pose_vec不是一个包含6个元素的向量，报错
        ROS_INFO("initial_pose is not a vector with 6 elements, what(): %s", ex.what()); 
    }

    std::string pointcloud_topic;
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/livox/points");
    pointcloud_sub_ = nh.subscribe(pointcloud_topic,10,pointcloudCallback);
    // pointcloud_sub_ = nh.subscribe(pointcloud_topic,10,pointCloudCallback_test);
    ROS_INFO("pointcloud_topic: %s\n",pointcloud_topic.c_str());


    tf_publisher_thread_ = std::make_unique<std::thread>([]() {
        ros::Rate rate(100);
        tf::TransformBroadcaster tf_broadcaster_;
        if(SINGLE_RUN){ // 快速调试
            tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom_transform, 
            ros::Time::now(), map_frame_id_, odom_frame_id_));

            // cloud_ == nullptr || fused_Cloud == nullptr || align_point == nullptr
            while(cloud_ == nullptr || fused_Cloud == nullptr || align_point == nullptr){
                // ROS_INFO("WAITING...\n");
                std::cout << "WAITING...\n" << std::endl;
                if(cloud_ == nullptr){
                    std::cout << "cloud_ == nullptr\n";
                }
                if(fused_Cloud == nullptr){
                    std::cout << "   fused_Cloud == nullptr\n";
                }
                if(align_point == nullptr){
                    std::cout << "   align_point == nullptr\n" << std::endl;
                }
                std::cout << "frame_count = " << frame_count << std::endl;
            }
            pclViewer(cloud_, fused_Cloud, align_point);
        }else{
            while (ros::ok()) {
                // 通过tf_broadcaster_对象发送TF变换信息到TF树中
                tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_odom_transform, 
                    ros::Time::now(), map_frame_id_, odom_frame_id_));
                rate.sleep();
            }
        }
    });
    ros::spin();

    ROS_INFO("icp_registration: initialized\n");
    return 0;  
}

// 修改前：const sensor_msgs::PointCloud2ConstPtr &msg
// 参考接收点云回调函数格式：const sensor_msgs::PointCloud2ConstPtr input

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if(SINGLE_RUN && run_once){
        return;
    }
    if(frame_count < frame_num){
        // 将ROS的PointCloud2消息转换为PCL的PointCloud对象
        // 打印点云大小
        size_t point_sz = msg->height * msg->width;
        // ROS_INFO("point msg size: %lu",point_sz);
        std::cout << "point msg size:" << point_sz << std::endl;
        pcl::fromROSMsg(*msg, *cloud_in_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rt_(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_rt_ = rotatingPointcloud(cloud_in_);
        *cloud_in_ = *cloud_rt_;
        first_scan_ = false;
        std::cout << "frame_count = " << frame_count << std::endl;
        if(frame_count == 0){
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.pose.pose = initial_pose_;
            *fused_Cloud = *cloud_in_;
           
        }else{
            *fused_Cloud += *cloud_in_;
        }
        frame_count ++;
        
    }else if(frame_count == frame_num){
        pointcloud_ready = true;

        Eigen::Matrix4d initial_guess;
        initial_guess << 1,0,0,0,
                        0,1,0,0,
                        0,0,1,0,
                        0,0,0,1;

        Eigen::Matrix4d map_to_laser = multiAlignSync(fused_Cloud, initial_guess);
        if (!success_) {
            // map_to_laser是配准修正后的机器人位移，initial_guess是初始坐标，也就是地图原点到机器人初始位置的位移
            // 如果匹配不成功，
            map_to_laser = initial_guess;
            // ROS_INFO("ICP failed!\n");
            std::cout << "ICP failed!" << std::endl;
        }
        if(success_){
            map_to_laser = matrixCorrection(map_to_laser);
            map_to_odom_transform = mapMatrixToTransform(map_to_laser);
            // ROS_INFO("CHANGE TF!");
            std::cout << "CHANGE TF!" << std::endl;
        }
        frame_count = 0;
        run_once = true;
    }
    
}



Eigen::Matrix4d multiAlignSync(PointCloudXYZI::Ptr source, const Eigen::Matrix4d &init_guess){
    static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    };
    success_ = false;
    Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
    Eigen::Vector3d rpy = rotate2rpy(rotation);
    Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
    std::vector<Eigen::Matrix4f> candidates;
    Eigen::Matrix4f temp_pose;

    // ROS_INFO("initial guess: %f, %f, %f, %f, %f, %f",xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -yaw_offset_; k <= yaw_offset_; k++) {
                Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                                    xyz(2));
                Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                        Eigen::Vector3f::UnitZ());
                temp_pose.setIdentity();
                temp_pose.block<3, 3>(0, 0) =
                    (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
                temp_pose.block<3, 1>(0, 3) = pos;
                candidates.push_back(temp_pose);
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
        new pcl::PointCloud<pcl::PointXYZI>);

    // ROS_INFO("pointcloud source size: %ld\n",source->points.size());
    if(source->points.size()<=0){
        // ROS_INFO("PointCloudXYZI::Ptr source IS EMPTY !!!\n");
        std::cout << "PointCloudXYZI::Ptr source IS EMPTY !!!\n" << std::endl;

    }
    voxel_rough_filter_.setInputCloud(source);
    voxel_rough_filter_.filter(*rough_source);
    voxel_refine_filter_.setInputCloud(source);
    voxel_refine_filter_.filter(*refine_source);

    PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
    PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);

    // PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);
    align_point = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    

    Eigen::Matrix4f best_rough_transform;
    double best_rough_score = 10.0;
    bool rough_converge = false;
    auto tic = std::chrono::system_clock::now();
    for (Eigen::Matrix4f &init_pose : candidates) {
        icp_rough_.setInputSource(rough_source_norm);
        icp_rough_.align(*align_point, init_pose);
        if (!icp_rough_.hasConverged()) continue;
        double rough_score = icp_rough_.getFitnessScore();
        if (rough_score > 2 * thresh_) continue;
        if (rough_score < best_rough_score) {
            best_rough_score = rough_score;
            rough_converge = true;
            best_rough_transform = icp_rough_.getFinalTransformation();
        }
    }

    if (!rough_converge) return Eigen::Matrix4d::Zero();

    icp_refine_.setInputSource(refine_source_norm);
    icp_refine_.align(*align_point, best_rough_transform);
    score_ = icp_refine_.getFitnessScore();

    // showPointCloud(align_point,"align_point",255,0,0);

    if (!icp_refine_.hasConverged()) return Eigen::Matrix4d::Zero();
    if (score_ > thresh_) return Eigen::Matrix4d::Zero();
    success_ = true;
    auto toc = std::chrono::system_clock::now();
    std::chrono::duration<double> duration = toc - tic;
    ROS_INFO("align used: %f ms\n", duration.count() * 1000);
    std::cout << "align used: " << duration.count() * 1000 << " ms\n" << std::endl;
    ROS_INFO("score: %f\n", score_);
    std::cout << "score: " << score_<< std::endl;


    return icp_refine_.getFinalTransformation().cast<double>();
}


// 给点云添加法线
PointCloudXYZIN::Ptr addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
    normalEstimator.setSearchMethod(searchTree);
    normalEstimator.setKSearch(15);
    normalEstimator.compute(*normals);
    PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
    pcl::concatenateFields(*cloud, *normals, *out);
    return out;
}


tf::Transform mapMatrixToTransform(const Eigen::Matrix4d& transformation_matrix) {
    tf::Transform transform;
    Eigen::Matrix3d rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
    tf::Matrix3x3 tf_rotation(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                              rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                              rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    tf::Quaternion tf_quaternion;
    tf_rotation.getRotation(tf_quaternion);
    transform.setRotation(tf_quaternion);
    tf::Vector3 tf_translation(transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3));
    transform.setOrigin(tf_translation);
    return transform;
}

PointCloudXYZI::Ptr rotatingPointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    PointCloudXYZI::Ptr out(new PointCloudXYZI);
    pcl::transformPointCloud (*cloud, *out, transformation_matrix_);
    return out;
}


Eigen::Matrix4d matrixCorrection(const Eigen::Matrix4d& transformation_matrix){
    // 将变换矩阵按照xz轴形成的面镜像对称
    Eigen::Matrix4d result = transformation_matrix;
    // 平移量镜像
    result (1, 3) = -result (1, 3);
    // 旋转量镜像
    result (0, 1) = -result (0, 1);
    result (1, 0) = -result (1, 0);
    return result;
}

void pclViewer(PointCloudXYZI::Ptr prior_point_cloud, PointCloudXYZI::Ptr pointcloud_before_tf, PointCloudXYZIN::Ptr pointcloud_after_tf){
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // Create two vertically separated viewports
    // 创建两个垂直分离的视图
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_in_color_h (
        prior_point_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (prior_point_cloud, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (prior_point_cloud, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v1"); // 设置点的大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v2"); // 设置点的大小


    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_tr_color_h (pointcloud_before_tf, 180, 30, 30);
    viewer.addPointCloud (pointcloud_before_tf, cloud_tr_color_h, "cloud_tr_v1", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_tr_v1"); // 设置点的大小

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> cloud_icp_color_h (pointcloud_after_tf, 20, 180, 20);
    viewer.addPointCloud (pointcloud_after_tf, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_icp_v2"); // 设置点的大小

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nRed: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    
    std::string icp_txt = "White: Original point cloud\nGreen: ICP aligned point cloud   score: " + std::to_string(score_);
    viewer.addText (icp_txt, 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::string icp_txt_m = "x="+std::to_string(map_to_odom_.transform.translation.x)+" y="+std::to_string(map_to_odom_.transform.translation.y)+" z="+std::to_string(map_to_odom_.transform.translation.x)+"\n"+
                            "rx="+std::to_string(map_to_odom_.transform.rotation.x)+" ry="+std::to_string(map_to_odom_.transform.rotation.y)+" z="+std::to_string(map_to_odom_.transform.rotation.z)+" rw="+std::to_string(map_to_odom_.transform.rotation.w);
    
    viewer.addText (icp_txt_m, 10, 45, 20, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_3", v2);
    // score_

    // std::stringstream ss;
    // ss << iterations;
    // std::string iterations_cnt = "ICP iterations = " + ss.str ();
    // viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    // viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100); // 100ms的延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 或者使用PCL的sleep函数
    }

}