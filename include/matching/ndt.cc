#include "./ndt.h"

namespace sad {


void NDT3D::set_Ndt(const double &eps, const int &max_iteration, const double &voxel_size, const double &min_eff_rate){
    eps_ = eps;
    max_iteration_ = max_iteration;
    voxel_size_ = voxel_size;
    min_eff_rate_ = min_eff_rate;
    inv_voxel_size_ = 1.0 / voxel_size_;
}
    
void NDT3D::SetTarget(CloudPtr target) {
    target_ = target;

    if(using_pcl_ndt){
        ndt_pcl_.setInputTarget(target);
    }
    else{
        BuildVoxels();
        // 计算点云中心
        target_center_ = std::accumulate(target->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                        [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                                        target_->size();
    }
}

void NDT3D::SetSource(CloudPtr source) {
    source_ = source;
    source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                    [](const Vec3d& c, const PointType& pt) -> Vec3d { return c + ToVec3d(pt); }) / source_->size();
}

double NDT3D::AlignWithLocalMap(CloudPtr scan, SE3 pose) 
{
    std::chrono::time_point<std::chrono::system_clock> t1, t2;
    t1 = std::chrono::system_clock::now();

    if(using_pcl_ndt){
        ndt_pcl_.setInputSource(scan);
    }
    else{
        SetSource(scan);
    }
    // final_effective_pts_ = 0;
    CloudPtr cloud_align_(new PointCloudType());
    if(using_pcl_ndt){
        ndt_pcl_.align(*cloud_align_, pose.matrix().cast<float>());
        pose_align_ = Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
    }
    else{
        AlignNdt(pose);
        pose_align_ = pose;
        pcl::transformPointCloud(*scan, *cloud_align_, pose_align_.matrix().cast<float>());
    }

    t2 = std::chrono::system_clock::now();
    double align_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

    ROS_INFO("scan size: %d", scan->size());
    ROS_INFO("Number of iterations: %d", iterations);
    ROS_INFO("align_time: %f ms", align_time);

    double final_effective_rate = final_effective_pts_/static_cast<double>(scan->size());
    return final_effective_rate;

}

void NDT3D::SetGtPose(const SE3& gt_pose) {
    gt_pose_ = gt_pose;
    gt_set_ = true;
}

FullCloudPtr NDT3D::Undistort(FullCloudPtr scan, std::vector<NavStated> imu_states_, SE3 pose_end) 
{
    SE3 Ti = pose_end;
    // 将所有点转到最后时刻状态上
    std::for_each(
        std::execution::par_unseq,
        // 并行的执行循环，多个线程上同时处理这些循环，加快计算速度 
        scan->points.begin(), scan->points.end(), [&](auto &pt) {
        NavStated match;
        // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
        math::PoseInterp<NavStated>
        (
            pt.time,
            imu_states_, // 这一帧点云期间的IMU姿态，用于插值
            [](const NavStated &s) { return s.timestamp_; },
            [](const NavStated &s) { return s.GetSE3(); }, 
            Ti, 
            match
        );
        // Lambda表达式作函数参数

        Vec3d pi = ToVec3d(pt);
        Vec3d p_compensate = TIL_.inverse() * pose_end.inverse() * Ti * TIL_ * pi;

        pt.x = p_compensate(0);
        pt.y = p_compensate(1);
        pt.z = p_compensate(2);
        }
    );

    return scan;
}

void NDT3D::BuildVoxels() {
    // 将点云地图划分为体素
    assert(target_ != nullptr);
    assert(target_->empty() == false);
    grids_.clear();

    std::vector<size_t> index(target_->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });
    // 填充索引

    std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
        auto pt = ToVec3d(target_->points[idx]);
        auto key = (pt * inv_voxel_size_).cast<int>();
        if (grids_.find(key) == grids_.end()) {
            grids_.insert({key, {idx}});
        } else {
            grids_[key].idx_.emplace_back(idx);
        }
    });

    /// 计算每个体素中的均值和协方差
    std::for_each(std::execution::par_unseq, grids_.begin(), grids_.end(), [this](auto& v) {
        if (v.second.idx_.size() > min_pts_in_voxel_) {
            // 要求至少有5个点
            math::ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                                    [this](const size_t& idx) { return ToVec3d(target_->points[idx]); });
            // SVD 检查最大与最小奇异值，限制最小奇异值

            Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }

            if (lambda[2] < lambda[0] * 1e-3) {
                lambda[2] = lambda[0] * 1e-3;
            }

            Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

            // v.second.info_ = (v.second.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免出nan
            v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        }
    });

    /// 删除点数不够的
    for (auto iter = grids_.begin(); iter != grids_.end();) {
        if (iter->second.idx_.size() > min_pts_in_voxel_) {
            iter++;
        } else {
            iter = grids_.erase(iter);
        }
    }
}

bool NDT3D::AlignNdt(SE3& init_pose) {
    // std::cout << "target_size: " << target_->size() << std::endl;
    // std::cout << "source_size: " << source_->size() << std::endl;

    assert(grids_.empty() == false);

    SE3 pose = init_pose;
    if (remove_centroid_) {
        pose.translation() = target_center_ - source_center_;  // 设置平移初始值
        std::cout << "init trans set to " << pose.translation().transpose() << std::endl;
    }

    // 对点的索引，预先生成
    int num_residual_per_point = 1;
    if (nearby_type_ == NearbyType::NEARBY6) {
        num_residual_per_point = 7;
    }

    std::vector<int> index(source_->points.size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // 并发代码
    int total_size = index.size() * num_residual_per_point;
    int iter;
    int effective_num = 0;
    for (iter = 0; iter < max_iteration_; ++iter) {
        std::vector<bool> effect_pts(total_size, false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
        std::vector<Vec3d> errors(total_size);
        std::vector<Mat3d> infos(total_size);

        // gauss-newton 迭代 最近邻，可以并发
        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
            auto q = ToVec3d(source_->points[idx]);
            Vec3d qs = pose * q;  // 转换之后的q

            // 计算qs所在的栅格以及它的最近邻栅格
            Vec3i key = (qs * inv_voxel_size_).cast<int>();

            for (int i = 0; i < nearby_grids_.size(); ++i) {
                auto key_off = key + nearby_grids_[i];
                auto it = grids_.find(key_off);
                int real_idx = idx * num_residual_per_point + i;
                if (it != grids_.end()) {
                    auto& v = it->second;       // voxel体素，包含均值和方差
                    Vec3d e = qs - v.mu_;       // 计算误差，变换后的坐标减去均值
                    
                    // check chi2 th
                    double res = e.transpose() * v.info_ * e;
                    if (std::isnan(res) || res > res_outlier_th_) {
                        // std::cout << "res: " <<res << std::endl;
                        effect_pts[real_idx] = false;
                        continue;
                    }

                    // build residual
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = Mat3d::Identity();

                    jacobians[real_idx] = J;
                    errors[real_idx] = e;
                    infos[real_idx] = v.info_;
                    effect_pts[real_idx] = true;
                } 
                else {
                    effect_pts[real_idx] = false;
                }
            }
        });

        // 累加Hessian和error,计算dx
        // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
        double total_res = 0;

        Mat6d H = Mat6d::Zero();
        Vec6d err = Vec6d::Zero();
        // std::cout << "effect_pts size: " << effect_pts.size() << std::endl;
        for (int idx = 0; idx < effect_pts.size(); ++idx) {
            if (!effect_pts[idx]) {
                continue;
            }

            total_res += errors[idx].transpose() * infos[idx] * errors[idx];
            effective_num++;

            H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
            err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
        }

        if (effective_num < min_effective_pts_) {
            std::cout << "effective num too small: " << effective_num << std::endl;
            return false;
        }

        Vec6d dx = H.inverse() * err;
        pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();

        // 更新
        // std::cout << "iter: " << iter << ", total res: " << total_res << ", eff: " << effective_num
        //           << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
        //           << ", dx: " << dx.transpose() << std::endl;
        ROS_INFO("[%f %f %f]", pose.translation()(0), pose.translation()(1), pose.translation()(2));


        if (gt_set_) {
            double pose_error = (gt_pose_.inverse() * pose).log().norm();
            std::cout << "iter " << iter << " pose error: " << pose_error << std::endl;
        }

        if (dx.norm() < eps_) {  // ! dx作为评价的指标?
            // std::cout << "converged, dx = " << dx.transpose() << std::endl; 
            break;
        }
        
    }

    final_effective_pts_ = effective_num;
    iterations = iter;
    init_pose = pose;
    return true;
}

void NDT3D::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}


}