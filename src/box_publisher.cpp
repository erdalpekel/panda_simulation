#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const double PANDA_ARM_TO_HAND_OFFSET = 0.12;
static const double PANDA_HAND_TO_FINGER_OFFSET = 0.04;
ros::Publisher gazebo_model_state_pub;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;

void jointStatesCallback(const sensor_msgs::JointState &joint_states_current)
{
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  std::vector<double> joint_states;
  for (size_t i = 0; i < joint_states_current.position.size() - 2; ++i)
  {
    joint_states.push_back(joint_states_current.position[i + 2]);
  }
  kinematic_state->setJointGroupPositions(joint_model_group, joint_states);

  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  double end_effector_z_offset = PANDA_ARM_TO_HAND_OFFSET + PANDA_HAND_TO_FINGER_OFFSET;
  Eigen::Affine3d tmp_transform(Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, end_effector_z_offset)));

  Eigen::Affine3d newState = end_effector_state * tmp_transform;

  geometry_msgs::Pose pose;
  pose.position.x = newState.translation().x();
  pose.position.y = newState.translation().y();
  pose.position.z = newState.translation().z();

  Eigen::Quaterniond quat(newState.rotation());
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  ROS_DEBUG_STREAM("translation" << std::endl << newState.translation());
  ROS_DEBUG_STREAM("rotation" << std::endl << newState.rotation());

  gazebo_msgs::ModelState model_state;
  // This string results from the spawn_urdf call in the box.launch file argument: -model box
  model_state.model_name = std::string("box");
  model_state.pose = pose;
  model_state.reference_frame = std::string("world");

  gazebo_model_state_pub.publish(model_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_publisher_node");
  ros::NodeHandle node_handle;

  ros::Subscriber joint_states_sub = node_handle.subscribe("/joint_states", 1, jointStatesCallback);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));

  gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

  ros::spin();
  return 0;
}