#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>

namespace ros_utils {

tf::StampedTransform getTransformToTargetFrame(const tf::TransformListener &tf_listener, std::string target_frame,
                                               std::string source_frame, double wait_time = 1.0);

void transformAsMatrix(const tf::Transform &bt, Eigen::Matrix4f &out_mat);

} // namespace ros_utils