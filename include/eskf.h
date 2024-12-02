#ifndef ESKF_H
#define ESKF_H

#include "./common/eigen_types.h"
#include "./common/imu.h"
#include "./common/math_utils.h"
#include "./common/nav_state.h"

#include <iomanip>
#include <iostream>

namespace sad {
/**
 * 使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav
 * S 状态变量的精度，取float或double
 */
template <typename S = double>
class ESKF {
   public:
    /// 类型定义
    using SO3 = Sophus::SO3<S>;                     // 旋转变量类型
    using VecT = Eigen::Matrix<S, 3, 1>;            // 向量类型
    using Vec18T = Eigen::Matrix<S, 18, 1>;         // 18维向量类型（名义状态和误差状态）
    using Mat3T = Eigen::Matrix<S, 3, 3>;           // 3x3矩阵类型
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;  // 运动噪声（建模噪声和测量噪声，18*18）类型
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;      // 里程计噪声类型
    using GnssNoiseT = Eigen::Matrix<S, 6, 6>;      // GNSS噪声类型
    using Mat18T = Eigen::Matrix<S, 18, 18>;        // 18维方差类型
    using NavStateT = NavState<S>;                  // 整体名义状态变量类型

    double imu_dt_ = 0.005;         // 判断IMU测量间隔，
    bool update_bias_gyro_ = true;  // 是否更新陀螺bias
    bool update_bias_acce_ = true;  // 是否更新加计bias

    void initPose(const SE3& pose){
        if(!isInitPose){
            p_ = pose.translation();
            R_ = pose.rotationMatrix();
            isInitPose = 1;
        }
    }
    void SetCov(const VecT& bg, const VecT& ba, const VecT& acc_cov, const VecT& gyro_cov, const VecT& gravity = VecT(0, 0, -9.81)) {
        double bias_acc_var_ = 1e-4; // 加速度随机游走标准差
        double bias_gyro_var_ = 1e-6; // 陀螺仪随机游走标准差
        Q_.diagonal() << 0, 0, 0, 
                    acc_cov.array().sqrt(),   // 加速度的标准差
                    gyro_cov.array().sqrt(),  //陀螺仪标准差
                    bias_gyro_var_,bias_gyro_var_,bias_gyro_var_, 
                    bias_acc_var_,bias_acc_var_,bias_acc_var_,
                    0, 0, 0;
        bg_ = bg;
        ba_ = ba;
        g_ = gravity;
        cov_ = Mat18T::Identity() * 1e-4;
    }

 
    /// 使用IMU递推
    bool Predict(const IMU& imu);

    /**
     * 使用SE3进行观测
     * pose  观测位姿
     * trans_noise 平移噪声
     * ang_noise   角度噪声
     */
    bool ObserveSE3(const SE3& pose, double trans_noise = 1e-4, double ang_noise = 1e-4);

    /// 获取SE3 状态
    SE3 GetNominalSE3() const { return SE3(R_, p_); }
    NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }

    /// 设置协方差
    void SetCov(const Mat18T& cov) { cov_ = cov; }

    /// 获取重力
    Vec3d GetGravity() const { return g_; }

   private:
    bool isInitPose = 0;
    double current_time_ = 0.0;
    /// Nominal state
    VecT p_ = VecT::Zero();
    VecT v_ = VecT::Zero();
    SO3 R_;
    // SO3的旋转矩阵，不用初始化，默认为单位阵
    VecT bg_ = VecT::Zero();
    VecT ba_ = VecT::Zero();

    VecT g_{0, 0, -9.8066};

    /// 误差状态
    Vec18T dx_ = Vec18T::Zero();

    /// 协方差阵
    Mat18T cov_ = Mat18T::Identity();
    // error state 初始时刻预测的方差
    // 初始时候为一个很小的值，单位对角阵：1e-4

    MotionNoiseT Q_ = MotionNoiseT::Zero();
     // 噪声的协方差（设置过程噪声）18*18维度
    // IMU测量噪声和建模噪声的协方差18*18
    /// IMU 测量与零偏参数


    /// 更新nominal state，重置error state
    void UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }
        if (update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }
        // 更新IMU的零偏
        g_ += dx_.template block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
        // 重置δx和error state的协方差
    }

    /// 对P阵进行投影，更新error state的协方差
    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }
};


// Nominal state 预测和Error state预测，仅仅与IMU数据有关
template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);
    // assert：断言，判断条件为假时，终止程序，报错

    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * imu_dt_) || dt < 0) {
        // 时间间隔过长（第一个IMU数据）或者过短
        std::cout << "skip this imu because dt_ = " << dt << std::endl;
        current_time_ = imu.timestamp_;
        // 更新当前时间
        return false;
        // 函数的返回值是布尔类型
    }

    // ----------------------------nominal state 预测 -----------------------------
    VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    // imu.acce_是IMU本体的加速度，本身含有重力项，R_是车体到导航系的旋转
    // IMU在世界坐标系中的位置和速度和方向，g_=-9.8066
    VecT new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    v_ = new_v;
    p_ = new_p;
    // Nominal state 就上面三个

    // -----------------------------error state 预测 -----------------------------
    // 根据nominal state 计算error state
    // 计算运动过程雅可比矩阵 F
    // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式
    Mat18T F = Mat18T::Identity();
    // 运动方程的线性化系数矩阵
    F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                         // p 对 v
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v对theta
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v 对 ba
    F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                        // v 对 g
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta 对 theta
    F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                        // theta 对 bg

    // mean and cov prediction
    // error state 的预测值和预测协方差
    dx_ = F * dx_;  // 这行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}

// ----------------------------------------观测部分-------------------------------------
template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    /// 既有旋转，也有平移
    /// 观测状态变量中的p, R，H为6x18，其余为零
    // 观测方程为6维度（3个平移和3个旋转），对状态x（维度为6*3=18）
    // 6个维度分别对18个状态分量求导，线性化之后的矩阵H维度为6*18
    Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Mat3T::Identity();  // P部分
    H.template block<3, 3>(3, 6) = Mat3T::Identity();  // R部分
    // 状态的次序为p,v,R,bg,ba,g,平移和旋转在第一个分块p（0，0起点，第三个分块（3，6起点）

    // 卡尔曼增益和更新过程
    Vec6d noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;

    Mat6d V = noise_vec.asDiagonal();
    // 观测的噪声

    Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();
    // 卡尔曼增益

    // 更新x和cov
    Vec6d innov = Vec6d::Zero();
    innov.template head<3>() = (pose.translation() - p_);  // 平移部分
    // 计算误差状态 δx = 观测 - 预测
    innov.template tail<3>() = (R_.inverse() * pose.so3()).log();  // 旋转部分

    dx_ = K * innov;

    // std::cout << "K: " << K << std::endl;
    cov_ = (Mat18T::Identity() - K * H) * cov_;

    UpdateAndReset();
    return true;
}
using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

}  // namespace sad

#endif