#ifndef MEASURE_SYNC
#define MEASURE_SYNC

#include "./cloud_convert_xyz.h"
#include "./common/imu.h"
#include <deque>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ros/ros.h>

namespace sad
{
    struct MeasureGroup
    {
        double lidar_begin_time_ = 0;   // 雷达包的起始时间
        double lidar_end_time_ = 0;     // 雷达的终止时间
        FullCloudPtr lidar_ = nullptr;  // 雷达全量点云，继承于PCL
        std::deque<IMUPtr> imu_;        // 上一时时刻到现在的IMU读数序列
    };

    class MessageSync
    {
    public:
        MessageSync(): conv_(new CloudConvert) {}
        
        bool flag = false;
        void ProcessIMU(IMUPtr imu){
            double timestamp = imu->timestamp_;
            if (timestamp < last_timestamp_imu_) {
                std::cout << "imu loop back, clear buffer";
                imu_buffer_.clear();
            }
            last_timestamp_imu_ = timestamp;
            imu_buffer_.emplace_back(imu);
        }

        void ProcessCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            if (msg->header.stamp.toSec() < last_timestamp_lidar_)
            {
                std::cout << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }
            FullCloudPtr cloud(new FullPointCloudType());
            conv_->Process(msg, cloud);                         // ! 将点云转化为全量点云，这里的转化似乎有点问题，参数是写死的
            
            lidar_buffer_.push_back(cloud);
            last_timestamp_lidar_ = msg->header.stamp.toSec();
            flag = Sync();
        }
        
        // 返回同步后的数据
        MeasureGroup getSyncResult() { return measures_; }

    private:
        // int sync_num_imu = 0;
        bool Sync();                                    // 尝试同步IMU与激光数据，成功时返回true

        std::shared_ptr<CloudConvert> conv_ = nullptr;  // 点云转换

        std::deque<FullCloudPtr> lidar_buffer_;         // 雷达数据缓冲
        std::deque<IMUPtr> imu_buffer_;                 // imu数据缓冲
        double last_timestamp_imu_ = -1.0;              // 最近imu时间
        double last_timestamp_lidar_ = 0;               // 最近lidar时间
        MeasureGroup measures_;

    };

    bool MessageSync::Sync() {

        if (lidar_buffer_.empty() || imu_buffer_.empty()) {return false;}
        measures_.imu_.clear();

        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = measures_.lidar_->points.front().time;
        measures_.lidar_end_time_ = measures_.lidar_->points.back().time;

        // 队列中第一帧的imu时间戳
        double imu_time = imu_buffer_.front()->timestamp_;

        // 如果队列中首帧IMU的时间戳大于雷达扫描结束时间，说明这帧点云采集期间没有IMU数据，移除雷达第一帧，返回
        if(imu_time > measures_.lidar_end_time_){
            lidar_buffer_.pop_front();
            return false;
        }

        while (!imu_buffer_.empty() && imu_time < measures_.lidar_end_time_) 
        {
            imu_time = imu_buffer_.front()->timestamp_;
            if(imu_time < measures_.lidar_begin_time_){ //小于开始扫描的时间，无效数据
                imu_buffer_.pop_front();
                continue;
            }
            
            if(imu_time < measures_.lidar_end_time_){
                measures_.imu_.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
                // sync_num_imu ++;
            }
            else{
                break;
            }
        }
        lidar_buffer_.pop_front();
        // std::cout << "sync_num_imu: " << sync_num_imu << std::endl;
        // sync_num_imu = 0;
        return true;
        
    }

} // namespace sad

#endif