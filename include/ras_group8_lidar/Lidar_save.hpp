#pragma once

#include <math.h>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

namespace ras_group8_lidar {

class Lidar
{
public:
  Lidar(ros::NodeHandle& node_handle);
  virtual ~Lidar();

private:
  bool readParameters();
  void lidarCallback(const sensor_msgs::PointCloud& msg);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber lidar_point_cloud_subscriber_;
  ros::Publisher lines_publisher_;
  
  /* Parameters
   */
  std::string lidar_point_cloud_topic_;

  /* Variables
   */
  double distance_threshold_;


  int s_left_start_;
  int s_left_stop_;
  int s_right_start_;
  int s_right_stop_;
  int s_front_start_;
  int s_front_stop_;

  nav_msgs::Path lines_;

};

} /* namespace */
