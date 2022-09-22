#include <panda_safety/SphereListVisualizer.h>

namespace nodes {

SphereListVisualizer::SphereListVisualizer(const ros::NodeHandle &node_handle,
                                           const ros::NodeHandle &private_node_handle, ros::Rate rate)
    : node_handle(node_handle), private_node_handle(private_node_handle), rate(rate) {
  this->init();
}

void SphereListVisualizer::init() {
  if (!private_node_handle.getParam(constants::CAMERA_ID, camera_id)) {
    throw std::runtime_error(
        "Parameter \"camera\" was not set when starting script! Specify which camera you are starting!");
  }

  service = node_handle.advertiseService(constants::service_endpoints::SPHERE_LIST_DISPLAY + camera_id,
                                         &SphereListVisualizer::sphereListDisplayCallback, this);
  ROS_INFO("Created service server");

  marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker_" + camera_id, 10);

  float f = 0.0f;
  while (ros::ok()) {
    if (!points_frame_id.empty()) {
      visualization_msgs::Marker points;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      points.header.frame_id = points_frame_id;
      points.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      points.ns = "points_" + camera_id;
      points.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      points.type = visualization_msgs::Marker::SPHERE_LIST;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      points.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      points.pose.position.x = 0;
      points.pose.position.y = 0;
      points.pose.position.z = 0;
      points.pose.orientation.x = 0.0;
      points.pose.orientation.y = 0.0;
      points.pose.orientation.z = 0.0;
      points.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      points.scale.x = 0.004;
      points.scale.y = 0.004;
      points.scale.z = 0.004;

      // Set the color -- be sure to set alpha to something non-zero!
      points.color.g = 1.0f;
      points.color.a = 1.0;

      ROS_DEBUG_STREAM("collision objects size: " << sphere_list.size());
      points.points = sphere_list;

      // Publish the marker
      marker_pub.publish(points);
    }

    rate.sleep();
    ros::spinOnce();
  }
}

bool SphereListVisualizer::sphereListDisplayCallback(panda_msgs::SphereListDisplayMsgRequest &req,
                                                     panda_msgs::SphereListDisplayMsgResponse &res) {
  ROS_INFO("In service callback");
  ROS_INFO_STREAM("Collision points size: " << req.sphere_list.size());
  sphere_list = req.sphere_list;
  points_frame_id = req.frame_id;

  return true;
}

} // namespace nodes

int main(int argc, char **argv) {
  ros::init(argc, argv, constants::nodes::COLLISION_VISUALIZER);
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");
  nodes::SphereListVisualizer node(node_handle, private_node_handle, 30);

  return 0;
}