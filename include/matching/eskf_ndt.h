#ifndef ESKF_NDT_H
#define ESKF_NDT_H

#pragma once
#include "ndt.h"
#include "../eskf.h"
#include "../measure_sync.h"
#include "../static_init_imu.h"
#include "../cloud_convert_xyz.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <filesystem>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <ros/ros.h>
using namespace std;

namespace sad {
class ESKF_NDT {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ESKF_NDT(){
        cloud_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }
    ~ESKF_NDT() = default;

    void init(ros::NodeHandle& nh);
    SE3 GetPoseAlign();  // eskf_ndt 最终位姿

    bool sysOK();
    bool measure_sync_OK();
    void run();     // 执行一次配准和观测
    void save_path(const string& pose_save_path, const int& save_mode);
    void publish_msg();
    void DebugPrint();

    string lidar_topic = "/points_raw";
    string imu_topic = "/IMU_data";
    string map_topic = "/local_map";
    string pcd_file_path = "";
    int init_time_seconds = 4;
    int imu_hz = 250;

    double min_eff_rate = 0.9;
    
    vector<double> ext_trans;
    vector<double> ext_rotation;

    int max_iteration = 20;
    double esp = 0.005;
    double voxel_size = 1.0;

    bool using_imu = true;

    ros::Time current_time;

    MessageSync sync_;                              // 雷达与点云信息同步器                     
    std::deque<MeasureGroup> measure_sync_buffer_;  // 同步后测量信息队列，防止不能够实时处理而丢失信息 只有当imu_init后开始运行

private:
    void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void imu_callback(const sensor_msgs::Imu::Ptr &msg);
    void map_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void init_pose_callback_clicked(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void init_pose_callback_param(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber map_sub;
    ros::Subscriber init_pose_sub_clicked;
    ros::Subscriber init_pose_sub_param;
    
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    ros::Publisher yaw_pub;
    ros::Publisher ndt_map_pub;         //debug
    ros::Publisher undisorted_lidar_pub;

    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_msg;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map;

    bool poseInitOK = 0;         // 初始化是否完成
    bool poseGetOK = 0;
    int path_wait_pub = 0;

    StaticIMUInit imu_init_;    // imu初始化，用于获得IMU的各个参数

    ESKFD eskf_;
    NDT3D ndt_;
    SE3 TIL_;

    tf::StampedTransform body_velodyne_transform;

    bool isMapInit = 0;
    bool is_first_frame_ = true;        // 第一帧特殊处理    
    bool pub_map = false;               // debug

    std::mutex pose_mutex;
    SE3 init_pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    SE3 pose_final_= SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());  // 获取配准后的位姿
    std::vector<NavStated> imu_states_; // ESKF预测期间的状态
    std::vector<SE3> estimated_poses_;  // 所有估计出来的pose，用于记录轨迹和预测下一个帧
    CloudPtr cloud_undistory;           // 用于记录发布去畸变的点云
    
};


}  // namespace sad


#endif
