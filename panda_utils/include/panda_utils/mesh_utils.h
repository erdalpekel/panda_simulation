#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Utils/getopt.h>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

namespace mesh_utils {

typedef OpenMesh::TriMesh_ArrayKernelT<> MeshType;
// typedef OpenMesh::PolyMesh_ArrayKernelT<> MeshType;

std::vector<Eigen::Vector3f> readRobotMeshFiles(const std::string &robot_mesh_files_path,
                                                const std::vector<std::string> &robot_mesh_files,
                                                const std::vector<std::string> robot_links,
                                                const std::map<std::string, std::string> &file_to_link_mapping);

std::vector<Eigen::Vector3f> readSingleMeshFileVertices(const std::string &file_path);

Eigen::VectorXf filterVolumeInsides(const Eigen::VectorXf &volume, std::vector<int> dims, bool full26Connected);
int threeDimensionalToFlatIndex(std::vector<int> index3D, std::vector<int> dim);
std::vector<int> flatToThreeDimensionalIndex(int flatIndex, std::vector<int> dim);

} // namespace mesh_utils