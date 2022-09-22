#pragma once

#include <geometry_msgs/PointStamped.h>
#include <panda_msgs/SphereListDisplayMsg.h>
#include <panda_utils/constants.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace nodes {

class SphereListVisualizer {
public:
  SphereListVisualizer() = delete;
  ~SphereListVisualizer() = default;

  SphereListVisualizer(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle, ros::Rate rate);

private:
  // public ros node handle
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;
  std::string node_name_{constants::nodes::COLLISION_VISUALIZER};

  std::vector<geometry_msgs::Point> sphere_list;
  std::string points_frame_id, camera_id;

  // ROS communication related - variables
  ros::Rate rate;
  ros::ServiceServer service;
  ros::Publisher marker_pub;

  void init();

  // ROS communication related - methods
  bool sphereListDisplayCallback(panda_msgs::SphereListDisplayMsgRequest &req,
                                 panda_msgs::SphereListDisplayMsgResponse &res);
};

} // namespace nodes
