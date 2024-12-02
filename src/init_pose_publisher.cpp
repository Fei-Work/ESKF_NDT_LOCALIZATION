#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "init_pose_pub");
    ros::NodeHandle nh;
    ros::Publisher init_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/init_pose", 1);
    geometry_msgs::PoseStamped init_pose;

    vector<double> init_position(3);
    vector<double> init_euler(3);   
    int max_init_time; 
    nh.param<vector<double>>("init_position", init_position, {0,0,0});
    nh.param<vector<double>>("init_euler", init_euler, {0, 0, 0});
    nh.param<int>("init_time_seconds", max_init_time, 5);

    init_pose.header.frame_id = "map";
    init_pose.header.stamp = ros::Time::now();
    init_pose.pose.position.x = init_position[0];
    init_pose.pose.position.y = init_position[1];
    init_pose.pose.position.z = init_position[2];

    tf2::Quaternion q;
    init_euler[0] = init_euler[0] * M_PI / 180.0; 
    init_euler[1] = init_euler[1] * M_PI / 180.0; 
    init_euler[2] = init_euler[2] * M_PI / 180.0; 

    q.setRPY(init_euler[2], init_euler[1], init_euler[0]); // Roll, Pitch, Yaw

    init_pose.pose.orientation.x = q.x();
    init_pose.pose.orientation.y = q.y();
    init_pose.pose.orientation.z = q.z();
    init_pose.pose.orientation.w = q.w();

    ros::Rate int_pose_rate(1);
    int i = 0;
    while (ros::ok()) {
        if(i >= max_init_time + 5){
            break;
        }
        init_pose.header.stamp = ros::Time::now();
        init_pose_publisher.publish(init_pose);
        int_pose_rate.sleep();
        i++;
    }
    return 0;
}