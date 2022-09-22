#include <panda_utils/ros_utils.h>

tf::StampedTransform ros_utils::getTransformToTargetFrame(const tf::TransformListener &tf_listener,
                                                          std::string target_frame, std::string source_frame,
                                                          double wait_time) {
  tf::StampedTransform transform;
  try {
    ROS_INFO_STREAM("lookup for source " << source_frame << " - target " << target_frame);
    tf_listener.waitForTransform(target_frame, source_frame, ros::Time::now(), ros::Duration(wait_time));
    tf_listener.lookupTransform(target_frame, source_frame, ros::Time(), transform);
  } catch (tf::TransformException ex) { ROS_ERROR("ERROR %s", ex.what()); }

  return std::move(transform);
}

// https://github.com/ros-perception/perception_pcl/blob/melodic-devel/pcl_ros/src/transforms.cpp
void ros_utils::transformAsMatrix(const tf::Transform &bt, Eigen::Matrix4f &out_mat) {
  double mv[12];
  bt.getBasis().getOpenGLSubMatrix(mv);

  tf::Vector3 origin = bt.getOrigin();

  out_mat(0, 0) = mv[0];
  out_mat(0, 1) = mv[4];
  out_mat(0, 2) = mv[8];
  out_mat(1, 0) = mv[1];
  out_mat(1, 1) = mv[5];
  out_mat(1, 2) = mv[9];
  out_mat(2, 0) = mv[2];
  out_mat(2, 1) = mv[6];
  out_mat(2, 2) = mv[10];

  out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
  out_mat(3, 3) = 1;
  out_mat(0, 3) = origin.x();
  out_mat(1, 3) = origin.y();
  out_mat(2, 3) = origin.z();
}