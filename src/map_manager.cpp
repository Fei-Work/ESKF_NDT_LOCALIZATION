#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

string map_path;

int main(int argc, char** argv)
{
  ROS_INFO("publish map node start !");
  ros::init(argc, argv, "map_manager");
  ros::NodeHandle nh;
  ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1); // 设置队列大小为1

  nh.param<string>("map_path", map_path, "");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud_map);
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*cloud_map, map_msg);
  map_msg.header.frame_id = "map";
  
  while (ros::ok())
  {
    ros::Duration(2).sleep();
    pcd_pub.publish(map_msg);
    // ROS_INFO("publishing global point map...");
    ros::Duration(5).sleep();
    ros::spinOnce();
  }
  return 0;
}
