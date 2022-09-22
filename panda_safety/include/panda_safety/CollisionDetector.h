#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H
#pragma once

#include <CL/cl.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ctime>
#include <geometry_msgs/Point.h>
#include <iomanip>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <panda_msgs/ReadMesh.h>
#include <panda_msgs/RobotStopMsg.h>
#include <panda_msgs/SphereListDisplayMsg.h>
#include <panda_safety/opencl_utils.h>
#include <panda_utils/constants.h>
#include <panda_utils/mesh_utils.h>
#include <panda_utils/ros_utils.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

template <typename T>
void readConfigParameter(ros::NodeHandle &node_handle, std::string parameter_key, T &parameter_value) {
  if (node_handle.getParam(parameter_key, parameter_value)) {
    return;
  } else {
    throw std::runtime_error(ros::this_node::getName() + " not able to read parameter " + parameter_key);
  }
}

class CollisionDetector {

public:
  CollisionDetector() = delete;
  ~CollisionDetector() = default;

  CollisionDetector(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

private:
  // public ros node handle
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;
  std::string node_name_{constants::nodes::COLLISION_DETECTOR};

  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  const robot_state::JointModelGroup *joint_model_group;

  std::vector<float> joint_position_trajectory;
  Eigen::Affine3d transform_panda_link0;
  Eigen::Matrix4f transform_depth_camera;
  Eigen::Isometry3d vtk_to_world_eigen;
  std::string camera_id;
  bool simulation, visualize;
  size_t index_link_start;
  std::vector<Eigen::Vector3f> link_vertices;
  std::vector<Eigen::Vector3f> link_vertices_old;

  std::string camera_depth_topic_suffix, camera_center_frame_suffix, trajectory_sub_goal_topic,
      trajectory_sub_result_topic, opencl_detection_kernel_collected_options,
      opencl_verification_kernel_collected_options, opencl_kernel_detection_path, camera_depth_topic,
      opencl_kernel_verification_path, path_panda_safety, camera_frame, reference_link_vtk;
  double end_effector_z_offset, origin_factor;
  int opencl_default_device, opencl_platform, use_opencl_device, result_length, num_mesh_vertices;
  std::size_t depth_resolution;
  std::size_t collision_depth, collision_width, collision_height;
  std::vector<float> max_box_pcl_max, max_box_pcl_min;
  std::vector<double> mesh_voxelation_spacing;

  // OpenCL specific variables
  cl::Context context;
  cl::Program program_detection;
  cl::Program program_verification;
  cl::Kernel detection_kernel;
  cl::Kernel verification_kernel;
  cl::CommandQueue opencl_queue;
  cl::NDRange workgroup_range;
  cl::Buffer transforms_buffer;
  cl::Buffer joint_position_trajectory_buffer;
  cl::Buffer transform_camera_buffer;
  cl::Buffer mesh_buffer;
  cl::Buffer num_vertices_buffer;
  cl::Buffer cloud_buffer;
  cl::Buffer result_buffer;
  cl::Buffer result_kernel_verification_buffer;
  cl::Image3D clImage;
  cl_float3 maximum_reachable_box;
  cl_float3 minimum_reachable_box;
  std::vector<cl_float4> transform_camera;

  std::vector<float> dimensions_reachable_box;
  std::map<std::uint64_t, const sensor_msgs::JointState> joint_states_buffer;
  std::deque<std::uint64_t> key_deque;

  // ROS communication related - variables
  tf::TransformListener tf_listener;
  ros::ServiceClient robot_control_stop;
  ros::ServiceClient sphere_list_display;
  ros::ServiceClient read_mesh_client;
  ros::Subscriber joint_states_sub;
  ros::Subscriber trajectory_sub;
  ros::Subscriber trajectory_result_sub;
  ros::Subscriber depth_camera_sub;

  void init();
  void initDepthResolution();
  void initResultBufferDevice();
  void initClImage3dBufferDevice();
  void setJointStatesArg(const sensor_msgs::JointState &joint_states_current);

  // ROS communication related - methods
  void collisionDetectionCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void jointStatesCallback(const sensor_msgs::JointState &joint_states_current);
  void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal &joint_trajectory_goal);
  void trajectoryResultCallback(const control_msgs::FollowJointTrajectoryActionResult &joint_trajectory_result);
};

#endif // COLLISION_DETECTOR_H
