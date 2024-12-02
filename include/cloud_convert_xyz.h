#ifndef CLOUD_CONVERT_XYZ
#define CLOUD_CONVERT_XYZ

/*
    输入的点云：只有xyz坐标，计算扫描点的yaw角，根据扫描角速度，恢复每个点的扫描时间
    绝对时间参考是点云的时间戳（点云扫描结束时间）
    扫描频率10Hz，扫描一圈转动角度363度，角速度: 3.63 degree/ms
    输出：带有时间的xyz点云
*/
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "./common/point_types.h"

#include <iostream>
#include <fstream>
#include <ros/ros.h>
using namespace std;

namespace sad {
class CloudConvert {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudConvert() {}
    ~CloudConvert() {}
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, FullCloudPtr &pcl_out);
private:
    void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    FullPointCloudType cloud_out_;  // 输出点云
    int point_filter_num_ = 2;      // 跳点
    double last_scan_time_ = 0.0;
};

void CloudConvert::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, FullCloudPtr &pcl_out) {
    VelodyneHandler(msg);
    *pcl_out = cloud_out_;
}

void CloudConvert::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize); // 将输入的激光雷达点云转化为velodyne点云格式
    double omega_l = 3.63;  // scan angular velocity  unit: degree/ms
    
    double yaw_angle = 0.0;
    double yaw_angle_temp = 0.0;
    double time_temp = msg->header.stamp.toSec();
    
    bool flag = true;
    for (int i = 0; i < plsize; i++) {
        FullPointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = msg->header.stamp.toSec();  // unit: s

        if (added_pt.getVector3fMap().norm() < 1.0 || added_pt.getVector3fMap().norm() > 70.0) {
            continue;
        }

        yaw_angle = atan2(added_pt.y, added_pt.x) * 57.29578;

        if(i == 0){
            // 点云的第一个点特殊处理，用于初始化，不会存入点云中
            yaw_angle_temp = yaw_angle;
            continue;
        }else{
            double delta_yaw = yaw_angle_temp - yaw_angle;
            if (delta_yaw < -180) {
                added_pt.time = time_temp + ((360.0 + delta_yaw) / omega_l)/1000.0;
                // 单位: s，范围: 0～0.1s
            } else { added_pt.time = time_temp + (delta_yaw / omega_l)/1000.0; }

            time_temp = added_pt.time;
            yaw_angle_temp = yaw_angle;
            
            if (i % point_filter_num_ == 0 && added_pt.time > last_scan_time_) {
                added_pt.time = added_pt.time - 0.1;
                // 雷达时间戳是扫描结束时的时间，因此将所有点的时间减去0.1s，保证时间的一致
                cloud_out_.points.push_back(added_pt);
            }
        } 
    }
    last_scan_time_ = time_temp;
}
} // namespace


#endif