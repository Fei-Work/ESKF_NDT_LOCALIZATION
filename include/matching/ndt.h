#ifndef NDT_H
#define NDT_H

// 传入配准点云和预测位姿，输出配准后的位姿，并建立地图
// #include "../ndt_3d.h"
#include "../common/math_utils.h"
#include "../common/eigen_types.h"
#include "../common/point_types.h"
#include "../common/lidar_utils.h"
#include "../common/nav_state.h"
#include <pcl/registration/ndt.h>
#include <pcl/common/transforms.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <execution>
#include <ros/ros.h>
using namespace std;

namespace sad {

class NDT3D {
   public: 
    enum class NearbyType {
        CENTER,   // 只考虑中心
        NEARBY6,  // 上下左右前后
    };

    bool using_pcl_ndt = false;
    
    NDT3D(){
        set_Ndt(0.005, 10, 1.0);

        // 构造函数内为成员变量初始化
        if(using_pcl_ndt){
            ndt_pcl_.setResolution(1.0);
            ndt_pcl_.setStepSize(0.1);
            ndt_pcl_.setTransformationEpsilon(0.001);
            ndt_pcl_.setMaximumIterations(10);
        }
        if (nearby_type_ == NearbyType::CENTER) {
            nearby_grids_.emplace_back(KeyType::Zero());
        } 
        else if (nearby_type_ == NearbyType::NEARBY6) {
            nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
        }
    }
    ~NDT3D(){};
    
    // 去畸变
    FullCloudPtr Undistort(FullCloudPtr scan, std::vector<NavStated> imu_states_, SE3 pose_end);
    double AlignWithLocalMap(CloudPtr scan, SE3 pose);

    // 添加点云和先验位姿
    CloudPtr global_map_ = nullptr;
    // 全局地图
    bool pub_global_map = false;
    SE3 pose_align_;

    // 配准后的位姿
    void set_Ndt(const double &eps, const int &max_iteration = 20, const double &voxel_size = 1.0, const double &min_eff_rate = 0.8);

    using KeyType = Eigen::Matrix<int, 3, 1>;  // 体素的索引

    struct VoxelData {
        VoxelData() {}
        VoxelData(size_t id) { idx_.emplace_back(id); }

        std::vector<size_t> idx_;      // 点云中点的索引
        Vec3d mu_ = Vec3d::Zero();     // 均值
        Mat3d sigma_ = Mat3d::Zero();  // 协方差
        Mat3d info_ = Mat3d::Zero();   // 协方差之逆
    };

    /// 设置目标的Scan: 点云地图
    void SetTarget(CloudPtr target);
    /// 设置被配准的Scan
    void SetSource(CloudPtr source);
    void SetGtPose(const SE3& gt_pose);


    int GetIterations(){return iterations;}
    
    /// 使用gauss-newton方法进行ndt配准
    bool AlignNdt(SE3& init_pose);

private:
    int max_iteration_ = 20;        // 最大迭代次数
    double voxel_size_ = 1.0;       // 体素大小
    double inv_voxel_size_ = 1.0;   //
    int min_effective_pts_ = 10;    // 最近邻点数阈值
    int min_pts_in_voxel_ = 5;      // 每个栅格中最小点数
    double eps_ = 0.01;             // 收敛判定条件
    double res_outlier_th_ = 10.0;  // 异常值拒绝阈值
    bool remove_centroid_ = false;  // 是否计算两个点云中心并移除中心？

    int final_effective_pts_ = 0;   // 在每次配准后最后的有效点数量
    double min_eff_rate_ = 0.85;

    NearbyType nearby_type_ = NearbyType::NEARBY6;

    
    int iterations = 0;
    void BuildVoxels();

    /// 根据最近邻的类型，生成附近网格
    void GenerateNearbyGrids();

    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Vec3d target_center_ = Vec3d::Zero();
    Vec3d source_center_ = Vec3d::Zero();

    SE3 gt_pose_;
    bool gt_set_ = false;

    std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_;  // 栅格数据
    std::vector<KeyType> nearby_grids_;                          // 附近的栅格

    SE3 TIL_;                                       // 激光雷达到IMU的外参
    CloudPtr local_map_ = nullptr;                  // 局部地图
    bool IsKeyframe(const SE3& current_pose);       // 判定是否为关键帧
    std::deque<CloudPtr> scans_in_local_map_;       // 局部点云地图中的所有关键帧
    std::vector<SE3> estimated_poses_;              // 所有估计出来的pose，用于记录轨迹和预测下一个帧
    SE3 last_kf_pose_;                              // 上一关键帧的位姿           
    pcl::NormalDistributionsTransform<PointType, PointType> ndt_pcl_;       // NDT对象
};


}  // namespace sad



#endif