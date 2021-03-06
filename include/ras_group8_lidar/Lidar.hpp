#pragma once

#include <math.h>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

namespace ras_group8_lidar {

class Lidar
{
public:
  Lidar(ros::NodeHandle& node_handle);
  virtual ~Lidar();

private:
  bool readParameters();
  void lidarCallback(const sensor_msgs::PointCloud& msg);
  visualization_msgs::Marker makeline(float x1, float y1, float x2, float y2,int id);

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;
  ros::Subscriber lidar_point_cloud_subscriber_;
  ros::Publisher lines_publisher_;
  ros::Publisher path_publisher_;
  tf::TransformListener listener;
  
  /* Parameters
   */
  std::string lidar_point_cloud_topic_;

  /* Variables
   */
  double distance_threshold_;
  double continue_threshold_;
  int minsize;


  int s_left_start_;
  int s_left_stop_;
  // int s_right_start_;
  // int s_right_stop_;
  // int s_front_start_;
  // int s_front_stop_;

  nav_msgs::Path lines_;

  visualization_msgs::MarkerArray walls;
  std::string target_frame;
  std::string laser_frame;

  


};

} /* namespace */
