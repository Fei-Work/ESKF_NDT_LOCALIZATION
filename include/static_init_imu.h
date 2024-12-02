#ifndef STATIC_INIT_IMU
#define STATIC_INIT_IMU

#include "./common/eigen_types.h"
#include "./common/imu.h"
#include "./common/math_utils.h"
#include <deque>

namespace sad {

/**
 * IMU水平静止状态下初始化器
 * 使用方法：调用AddIMU, 使用InitSuccess获取初始化是否成功
 * 成功后，使用各Get函数获取内部参数
 */
class StaticIMUInit {
   public:

    void Setting(double time_sec, int imu_queue){
        init_time_seconds_ = time_sec;
        init_imu_queue_max_size_ = imu_queue;
    }

    /// 添加IMU数据
    bool AddIMU(const IMU& imu);

    /// 判定初始化是否成功
    bool InitSuccess() const { return init_success_; }

    /// 获取各Cov, bias, gravity
    Vec3d GetCovGyro() const { return cov_gyro_; }
    Vec3d GetCovAcce() const { return cov_acce_; }
    Vec3d GetInitBg() const { return init_bg_; }
    Vec3d GetInitBa() const { return init_ba_; }
    Vec3d GetGravity() const { return gravity_; }

   private:
    double init_time_seconds_ = 4.0;     // 静止时间
    int init_imu_queue_max_size_ = 2000;  // 初始化IMU队列最大长度
    double max_static_gyro_var = 0.5;     // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;    // 静态下加计测量方差
    double gravity_norm_ = 9.81;          // 重力大小

    /// 尝试对系统初始化
    bool TryInit();
    bool init_success_ = false;       // 初始化是否成功
    Vec3d cov_gyro_ = Vec3d::Zero();  // 陀螺测量噪声协方差（初始化时评估）
    Vec3d cov_acce_ = Vec3d::Zero();  // 加计测量噪声协方差（初始化时评估）
    Vec3d init_bg_ = Vec3d::Zero();   // 陀螺初始零偏
    Vec3d init_ba_ = Vec3d::Zero();   // 加计初始零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力
    std::deque<IMU> init_imu_deque_;  // 初始化用的数据
    double current_time_ = 0.0;       // 当前时间
    double init_start_time_ = 0.0;    // 静止的初始时间
};


bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (init_success_){
        // 判断是否初始化完成
        return true;
    }

    if (init_imu_deque_.empty()) {
        // 记录初始静止时间
        init_start_time_ = imu.timestamp_;
    }

    // 记入初始化队列
    init_imu_deque_.push_back(imu);

    double init_time = imu.timestamp_ - init_start_time_;  // 初始化经过时间
    if (init_time > init_time_seconds_) {
        // 当采集数据大于静止要求时间时，启动初始化函数
        TryInit();
    }

    // 维持初始化队列长度
    while (init_imu_deque_.size() > init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
        // 当序列大于阈值时，移除掉最早的IMU数据
    }

    current_time_ = imu.timestamp_;
    return false;
}

bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // 计算均值和方差
    Vec3d mean_gyro, mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce_; });

    // 以acce均值为方向，取9.8长度为重力
    std::cout << "mean acce: " << mean_acce.transpose() << std::endl;
    gravity_ = -mean_acce / mean_acce.norm() * gravity_norm_;

    // 重新计算加计的协方差
    math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });

    // 检查IMU噪声
    if (cov_gyro_.norm() > max_static_gyro_var) {
        std::cout << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << max_static_gyro_var;
        return false;
    }

    if (cov_acce_.norm() > max_static_acce_var) {
        std::cout << "加计测量噪声太大" << cov_acce_.norm() << " > " << max_static_acce_var;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    std::cout << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro cov(方差) = " << cov_gyro_.transpose()
              << ", acce cov(方差) = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm() << std::endl;
    std::cout << "mean gyro: " << mean_gyro.transpose() << " mean acce: " << mean_acce.transpose() << std::endl;
    init_success_ = true;
    return true;
}

}  // namespace sad


#endif