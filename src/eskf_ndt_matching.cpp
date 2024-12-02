#include <ros/ros.h>
#include <csignal>

#include "../include/matching/eskf_ndt.h"
#include "../include/tool.h"
using namespace std;

bool flg_exit = false;

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    // sig_buffer.notify_all();
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "eskf_ndt_matching_node");
    ros::NodeHandle nh;    
    sad::ESKF_NDT ESKF_NDT;

    string pose_save_path;
    int save_mode;

    nh.param<string>("lidar_topic", ESKF_NDT.lidar_topic, "/points_raw");
    nh.param<string>("imu_topic", ESKF_NDT.imu_topic, "/IMU_data");
    nh.param<int>("init_time_seconds", ESKF_NDT.init_time_seconds, 4);
    nh.param<int>("imu_hz", ESKF_NDT.imu_hz, 250);
    nh.param<double>("min_eff_rate", ESKF_NDT.min_eff_rate, 0.85);

    nh.param<vector<double>>("L2I/extrinsic_T", ESKF_NDT.ext_trans, {0,0,0});
    nh.param<vector<double>>("L2I/extrinsic_R", ESKF_NDT.ext_rotation, {1,0,0,0,1,0,0,0,1});
    
    nh.param<double>("esp", ESKF_NDT.esp, 0.005);
    nh.param<int>("max_iteration", ESKF_NDT.max_iteration, 20);
    nh.param<double>("voxel_size", ESKF_NDT.voxel_size, 1.0);

    nh.param<string>("pose_save_path", pose_save_path, "./result/path.txt");
    nh.param<int>("save_mode", save_mode, 1);

    ROS_INFO("matching node start !");
    ROS_INFO("lidar_topic: %s", ESKF_NDT.lidar_topic.c_str());
    ROS_INFO("imu_topic: %s", ESKF_NDT.imu_topic.c_str());
    ROS_INFO("init_time_seconds: %d", ESKF_NDT.init_time_seconds);
    ROS_INFO("imu_hz: %d", ESKF_NDT.imu_hz);
    ROS_INFO("esp data: %f", ESKF_NDT.esp);
    ROS_INFO("max_iteration: %d", ESKF_NDT.max_iteration);
    ROS_INFO("min_eff_rate: %f", ESKF_NDT.min_eff_rate);
    ROS_INFO("voxel_size: %f", ESKF_NDT.voxel_size);
    ROS_INFO("Extrinsic Translation (T): [%f, %f, %f]", ESKF_NDT.ext_trans[0], ESKF_NDT.ext_trans[1], ESKF_NDT.ext_trans[2]);

    ESKF_NDT.init(nh);

    signal(SIGINT, SigHandle);
    ros::Rate matching_pub_rate(10);
    while(ros::ok())
    {
        if(flg_exit){break;}
        if(ESKF_NDT.measure_sync_OK()){
            ESKF_NDT.run();
            ESKF_NDT.DebugPrint();
        }
        ros::spinOnce();
        matching_pub_rate.sleep();
    }
    ESKF_NDT.save_path(pose_save_path, save_mode);
    return 0;
}
