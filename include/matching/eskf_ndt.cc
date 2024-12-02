#include "./eskf_ndt.h"


namespace sad {

void ESKF_NDT::init(ros::NodeHandle& nh){
    imu_init_.Setting(init_time_seconds, init_time_seconds*imu_hz); // ? 此处的imu——hz能否改为利用bag的时间戳格式
    ndt_.set_Ndt(esp, max_iteration, voxel_size, min_eff_rate);

    cloud_undistory.reset(new pcl::PointCloud<pcl::PointXYZI>());;
    Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_rotation);
    Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_trans);
    TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
    // SE3 TLI_ = TIL_.inverse();
    Eigen::Quaterniond rotation = TIL_.unit_quaternion();
    Eigen::Vector3d translation = TIL_.translation();
    body_velodyne_transform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
    body_velodyne_transform.setRotation(tf::Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));

    map_sub = nh.subscribe(map_topic, 10, &ESKF_NDT::map_callback, this);
    ros::Rate waiting_map_rate(1);
    while(isMapInit == 0){
        waiting_map_rate.sleep();
        ROS_INFO("WAITING FOR MAP TOPIC: %s", map_topic.c_str());
        ros::spinOnce();
    }

    init_pose_sub_clicked = nh.subscribe("/initialpose", 5, &ESKF_NDT::init_pose_callback_clicked, this);
    init_pose_sub_param = nh.subscribe("/init_pose", 5, &ESKF_NDT::init_pose_callback_param, this);
    lidar_sub = nh.subscribe(lidar_topic, 100, &ESKF_NDT::lidar_callback, this);
    imu_sub = nh.subscribe(imu_topic, 1000, &ESKF_NDT::imu_callback, this);    
        
    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
    undisorted_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/undisorted_points_raw", 1000);
    path_pub = nh.advertise<nav_msgs::Path>("/path_vlp", 1000);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1000);
    yaw_pub = nh.advertise<std_msgs::Float32>("/yaw_angle", 10);
}

SE3 ESKF_NDT::GetPoseAlign(){return ndt_.pose_align_;} 

bool ESKF_NDT::sysOK() {
    if(imu_init_.InitSuccess() && poseInitOK == 1){
    // if(imu_init_.InitSuccess()){
        return 1;
    }
    else{
        return 0;
    }
}

bool ESKF_NDT::measure_sync_OK(){
    return measure_sync_buffer_.size()>0;
}

void ESKF_NDT::run() {
    // 处理同步后的IMU消息
    ROS_INFO("measure_sync_buffer_: %d", measure_sync_buffer_.size());
    if (measure_sync_OK())
    {
        imu_states_.clear();
        MeasureGroup measure_sync_;
        measure_sync_ = measure_sync_buffer_.front();
        current_time = ros::Time(measure_sync_.lidar_end_time_);

        imu_states_.emplace_back(eskf_.GetNominalState());  // ! eskf要初始化位姿
        for (auto &imu : measure_sync_.imu_){
            eskf_.Predict(*imu);
            imu_states_.emplace_back(eskf_.GetNominalState());
        }

        // 点云去畸变  // ! 运动信息是如何使用的？
        FullCloudPtr cloud_undistory_full(new FullPointCloudType);
        cloud_undistory_full = ndt_.Undistort(measure_sync_.lidar_, imu_states_, eskf_.GetNominalSE3());

        cloud_undistory = ConvertToCloud(cloud_undistory_full);         //全量点云转

        ndt_.AlignWithLocalMap(cloud_undistory, eskf_.GetNominalSE3());
        eskf_.ObserveSE3(ndt_.pose_align_, 1e-4, 1e-4);
        pose_final_ = ndt_.pose_align_;

        measure_sync_buffer_.pop_front();
    }
    publish_msg();
}

void ESKF_NDT::init_pose_callback_clicked(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    if(!poseGetOK){
        Eigen::Matrix3d rotation_matrix;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);  // 自动加锁，作用域结束时自动解锁
            Eigen::Vector3d translation(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.4);
            Eigen::Quaterniond quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            rotation_matrix = quaternion.toRotationMatrix();
            init_pose = SE3(rotation_matrix, translation);
            poseGetOK = 1;
        }

        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
        ROS_INFO("set init position: [%f,%f,%f]", init_pose.translation()(0), init_pose.translation()(1), init_pose.translation()(2));
        ROS_INFO("Euler angles (RPY): Roll [%f], Pitch [%f], Yaw [%f]", euler_angles(2), euler_angles(1), euler_angles(0));
    }
}

void ESKF_NDT::init_pose_callback_param(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if(!poseGetOK){
        Eigen::Matrix3d rotation_matrix;
        {
            std::lock_guard<std::mutex> lock(pose_mutex);  // 自动加锁，作用域结束时自动解锁
            Eigen::Vector3d translation(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Eigen::Quaterniond quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            rotation_matrix = quaternion.toRotationMatrix();
            init_pose = SE3(rotation_matrix, translation);
        }
        poseGetOK = 1;
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
        ROS_INFO("set init position: [%f,%f,%f]", init_pose.translation()(0), init_pose.translation()(1), init_pose.translation()(2));
        ROS_INFO("Euler angles (RPY): Roll [%f], Pitch [%f], Yaw [%f]", euler_angles(2), euler_angles(1), euler_angles(0));
    }
}

void ESKF_NDT::lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    if(sysOK()){   
        sync_.ProcessCloud(msg);    // 导入点云信息
        if(sync_.flag){
            MeasureGroup msgroup = sync_.getSyncResult();
            pcl::transformPointCloud(*msgroup.lidar_, *msgroup.lidar_, TIL_.matrix());
            measure_sync_buffer_.push_back(msgroup);      //这里已经默认点云为低频信息，每次以一组点云时间为准
        }
    }
    else{
        if(!poseInitOK &&  poseGetOK){
            ROS_INFO("START INIT ndt matching");
            double max_rate = 0;
            SE3 max_rate_pose;
            SE3 try_pose;
            {
                std::lock_guard<std::mutex> lock(pose_mutex);  // 自动加锁，作用域结束时自动解锁
                try_pose = init_pose;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg, *cloud);   // 将 ROS 消息转换为 PCL 点云
            // ROS_WARN("0");
            pcl::transformPointCloud(*cloud, *cloud, TIL_.matrix());
            // ROS_WARN("1");
            ndt_.AlignWithLocalMap(cloud, try_pose);

            max_rate_pose = ndt_.pose_align_;
            ROS_INFO("INIT POSE: [%f, %f, %f]:", max_rate_pose.translation()(0), max_rate_pose.translation()(1), max_rate_pose.translation()(2));
            eskf_.initPose(max_rate_pose);
            poseInitOK = 1;
            ROS_INFO("FINISH INIT ndt matching");
        }
    }
}

void ESKF_NDT::imu_callback(const sensor_msgs::Imu::Ptr &msg){
    Vec3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    Vec3d acce(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    double timestamp = msg->header.stamp.toSec(); 

    if(ESKF_NDT::sysOK()){
        IMUPtr imuPtr = std::make_shared<sad::IMU>(timestamp, gyro, acce);
        sync_.ProcessIMU(imuPtr);
    }
    else{
        if(!imu_init_.InitSuccess()){
            sad::IMU imu = sad::IMU(timestamp, gyro, acce);
            imu_init_.AddIMU(imu);
            eskf_.SetCov(imu_init_.GetInitBg(), 
                        imu_init_.GetInitBa(), imu_init_.GetCovAcce(),
                        imu_init_.GetCovGyro(), imu_init_.GetGravity());
        }
    }
}

void ESKF_NDT::map_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    if(isMapInit == 0){
        isMapInit = 1;
        pcl::fromROSMsg(*msg, *cloud_map);
        ndt_.SetTarget(cloud_map);
        ROS_INFO("Get new local map");
        ROS_INFO("the number of map points: %d", cloud_map->size());
    }
}

void ESKF_NDT::save_path(const string& pose_save_path, const int& save_mode = 0){
    if(save_mode != 1 && save_mode != 0){
        return;
    }
    std::ofstream trajectory_file;
    trajectory_file.open(pose_save_path);  // 替换为你想保存的位置

    if (!trajectory_file.is_open()) {
        ROS_ERROR("Failed to open trajectory file!");
        return ;
    }

    if(save_mode == 0){
        trajectory_file << "time,translation_x,translation_y,translation_z,orientation_x,orientation_y,orientation_z,orientation_w" << std::endl;
    }
    if(save_mode == 1){
        trajectory_file << "translation_x,translation_y" << std::endl;
    }


    // 保存路径中的所有位姿数据
    for (const auto& pose_stamped : path_msg.poses) {

        double timestamp = pose_stamped.header.stamp.toSec();
        const auto& position = pose_stamped.pose.position;

        if(save_mode == 0){
            const auto& orientation = pose_stamped.pose.orientation;
            trajectory_file << timestamp << ","
                            << position.x << ","
                            << position.y << ","
                            << position.z << ","
                            << orientation.x << ","
                            << orientation.y << ","
                            << orientation.z << ","
                            << orientation.w << std::endl;
        }
        if(save_mode == 1){
            trajectory_file << position.x << ","
                            << position.y << std::endl;
        }
    }
    // 程序结束时关闭文件
    trajectory_file.close();
    ROS_INFO("Trajectory saved to %s", pose_save_path.c_str());
}

void ESKF_NDT::publish_msg(){
    if(pub_map){
        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_map, *map_msg_ptr);
        map_msg_ptr->header.stamp = current_time;
        map_msg_ptr->header.frame_id = "map";
        ndt_map_pub.publish(*map_msg_ptr);
        pub_map = false;
    }

    static tf::TransformBroadcaster broadcaster;
    tf::StampedTransform map_body_transform;

    SE3 current_pose = pose_final_;
    Eigen::Quaterniond current_q = current_pose.so3().unit_quaternion();
    map_body_transform.setRotation(tf::Quaternion(current_q.x(), current_q.y(), current_q.z(), current_q.w()));
    map_body_transform.setOrigin(tf::Vector3(current_pose.translation().x(), current_pose.translation().y(), current_pose.translation().z()));

    ros::Duration time_adjustment(0.001);  // 创建一个表示 1 毫秒的时间差
    ros::Time adjusted_time = current_time - time_adjustment;
    broadcaster.sendTransform(tf::StampedTransform(body_velodyne_transform, adjusted_time,  "/body", "/velodyne")); // ? temp
    broadcaster.sendTransform(tf::StampedTransform(map_body_transform, current_time, "/map", "/body"));

    if(path_wait_pub>10){
        geometry_msgs::PoseStamped pose_msg;

        tf::poseTFToMsg(map_body_transform, pose_msg.pose);

        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = current_time;

        path_msg.header.frame_id = "map";
        path_msg.header.stamp = current_time;

        double yaw = tf::getYaw(map_body_transform.getRotation());
        std_msgs::Float32 yaw_msg;
        yaw_msg.data = yaw; // 偏航角
        path_msg.poses.push_back(pose_msg);
        path_pub.publish(path_msg);
        pose_pub.publish(pose_msg);
        yaw_pub.publish(yaw_msg);
    }
    else{
        path_wait_pub++;
    }

    sensor_msgs::PointCloud2::Ptr cloud_undistory_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_undistory, *cloud_undistory_msg_ptr);
    cloud_undistory_msg_ptr->header.stamp = current_time;
    cloud_undistory_msg_ptr->header.frame_id = "body";
    undisorted_lidar_pub.publish(cloud_undistory_msg_ptr);
}

void ESKF_NDT::DebugPrint(){
    // 输出调试信息
    ROS_INFO("translation: [%f, %f, %f]", GetPoseAlign().translation().transpose()(0), GetPoseAlign().translation().transpose()(1), GetPoseAlign().translation().transpose()(2));
    ROS_INFO("pose: [%f, %f, %f, %f]", GetPoseAlign().so3().unit_quaternion().coeffs().transpose()(0), GetPoseAlign().so3().unit_quaternion().coeffs().transpose()(1), GetPoseAlign().so3().unit_quaternion().coeffs().transpose()(2), GetPoseAlign().so3().unit_quaternion().coeffs().transpose()(3));
    ROS_INFO("-------------------");
}

}