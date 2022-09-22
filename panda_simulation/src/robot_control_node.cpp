#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <panda_msgs/RobotStopMsg.h>
#include <panda_utils/constants.h>
#include <ros/ros.h>

static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const std::string APP_DIRECTORY_NAME = ".panda_simulation";
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm;

moveit_msgs::CollisionObject extractObstacleFromJson(Json::Value &root, std::string name) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  const Json::Value dimensions = root["dimensions"];
  ROS_INFO_STREAM("Extracted dimensions: " << dimensions);
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dimensions["x"].asDouble();
  primitive.dimensions[1] = dimensions["y"].asDouble();
  primitive.dimensions[2] = dimensions["z"].asDouble();

  const Json::Value position = root["position"];
  ROS_INFO_STREAM("Extracted position: " << position);

  const Json::Value orientation = root["orientation"];
  ROS_INFO_STREAM("Extracted orientation: " << orientation);
  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = orientation["w"].asDouble();
  box_pose.orientation.x = orientation["x"].asDouble();
  box_pose.orientation.y = orientation["y"].asDouble();
  box_pose.orientation.z = orientation["z"].asDouble();

  // MoveIt! planning scene expects the center of the object as position.
  // We add half of its dimension to its position
  box_pose.position.x = position["x"].asDouble() + primitive.dimensions[0] / 2.0;
  box_pose.position.y = position["y"].asDouble() + primitive.dimensions[1] / 2.0;
  box_pose.position.z = position["z"].asDouble() + primitive.dimensions[2] / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return std::move(collision_object);
}

bool stop(panda_msgs::RobotStopMsg::Request &request, panda_msgs::RobotStopMsg::Response &response) {
  move_group_arm->stop();
  ROS_INFO("STOP command fired with move_group <panda_arm>");

  return true;
}

int main(int argc, char **argv) {
  namespace fs = boost::filesystem;
  ROS_INFO("RUNNING robot_control_node");

  ros::init(argc, argv, "robot_control_node");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_group_arm = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_ARM);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
    sleep_t.sleep();
  }
  moveit_msgs::PlanningScene planning_scene;

  // read JSON files from ~/.panda_simulation
  fs::path home(getenv("HOME"));
  if (fs::is_directory(home)) {
    fs::path app_directory(home);
    app_directory /= APP_DIRECTORY_NAME;

    if (!fs::exists(app_directory) && !fs::is_directory(app_directory)) {
      ROS_WARN_STREAM(app_directory << " does not exist");

      // Create .panda_simulation directory
      std::string path(getenv("HOME"));
      path += "/.panda_core";
      ROS_INFO("Creating %s collision objects directory.", path);
      try {
        boost::filesystem::create_directory(path);
      } catch (const std::exception &) {
        ROS_ERROR("%s directory could not be created."
                  "Please create this directory yourself "
                  "if you want to specify collision objects.",
                  path.c_str());
        return -1;
      }
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    ROS_INFO_STREAM(app_directory << " is a directory containing:");
    for (auto &entry : boost::make_iterator_range(fs::directory_iterator(app_directory), {})) {
      ROS_INFO_STREAM(entry);

      std::ifstream file_stream(entry.path().string(), std::ifstream::binary);
      if (file_stream) {
        Json::Value root;
        file_stream >> root;

        moveit_msgs::CollisionObject collision_object = extractObstacleFromJson(root, entry.path().stem().string());
        collision_objects.push_back(collision_object);
      } else {
        ROS_WARN_STREAM("could not open file " << entry.path());
      }
    }

    // Publish the collision objects to the scene
    for (const auto &collision_object : collision_objects) {
      collision_object.header.frame_id = move_group_arm->getPlanningFrame();
      planning_scene.world.collision_objects.push_back(collision_object);
    }

    ROS_INFO_STREAM("# collision objects " << planning_scene.world.collision_objects.size());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ros::ServiceServer stop_service = node_handle.advertiseService(constants::service_endpoints::ROBOT_STOP, &stop);

    ROS_INFO("robot_control_node is ready");
    ros::waitForShutdown();

    return 0;
  }
}