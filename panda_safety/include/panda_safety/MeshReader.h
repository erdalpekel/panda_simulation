#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <panda_msgs/ReadMesh.h>
#include <panda_utils/constants.h>

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>
#include <vtkImageResize.h>
#include <vtkImageStencil.h>
#include <vtkMetaImageWriter.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPointData.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkSmartPointer.h>

namespace nodes {

class MeshReader {

public:
  MeshReader() = delete;
  ~MeshReader() = default;

  MeshReader(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle);

private:
  // public ros node handle
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;
  std::string node_name_{constants::nodes::MESH_READER};

  // config values loaded during startup

  // ROS communication related - variables
  ros::ServiceServer read_mesh_server;

  void init();
  bool readMeshCallback(panda_msgs::ReadMesh::Request &req, panda_msgs::ReadMesh::Response &res);
};

} // namespace nodes