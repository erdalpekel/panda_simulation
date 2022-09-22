#include <panda_utils/mesh_utils.h>

namespace mesh_utils {

std::vector<Eigen::Vector3f> readRobotMeshFiles(const std::string &robot_mesh_files_path,
                                                const std::vector<std::string> &robot_mesh_files,
                                                const std::vector<std::string> robot_links,
                                                const std::map<std::string, std::string> &file_to_link_mapping) {
  std::vector<Eigen::Vector3f> link_vertices;

  for (auto &file : robot_mesh_files) {
    std::string link_name;
    try {
      link_name = file_to_link_mapping.at(file);
    } catch (std::exception ex) {
      throw std::runtime_error(ros::this_node::getName() + " not able to read file link mapping " + file);
    }

    std::string file_path = robot_mesh_files_path + file;
    auto vertices_tmp = readSingleMeshFileVertices(file_path);
    link_vertices.insert(link_vertices.end(), std::make_move_iterator(vertices_tmp.begin()),
                         std::make_move_iterator(vertices_tmp.end()));
  }

  return link_vertices;
}

std::vector<Eigen::Vector3f> readSingleMeshFileVertices(const std::string &file_path) {
  std::vector<Eigen::Vector3f> vertices;
  MeshType mesh;
  if (!OpenMesh::IO::read_mesh(mesh, file_path)) {
    throw std::runtime_error(ros::this_node::getName() + " not able to read file path: " + file_path);
  } else {
    ROS_DEBUG_STREAM("Opened mesh file " << file_path);
    ROS_DEBUG_STREAM("# Vertices " << mesh.n_vertices());
    ROS_DEBUG_STREAM("# Edges " << mesh.n_edges());
    ROS_DEBUG_STREAM("# Faces " << mesh.n_faces());

    for (MeshType::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
      auto point = mesh.point(*v_it);
      // ROS_INFO_STREAM(point);
      vertices.push_back(Eigen::Vector3f(point[0], point[1], point[2]));
    }
  }

  return std::move(vertices);
}

int threeDimensionalToFlatIndex(std::vector<int> index3D, std::vector<int> dim) {
  return index3D[2] * dim[0] * dim[1] + index3D[1] * dim[0] + index3D[0];
}

std::vector<int> flatToThreeDimensionalIndex(int flatIndex, std::vector<int> dim) {
  std::vector<int> index = {0, 0, 0};
  Eigen::Vector3i dim_eigen(dim.at(0), dim.at(1), dim.at(2)), prod_coeff_per_dim;
  for (int i = 0; i < dim.size(); ++i)
    prod_coeff_per_dim(i) = dim_eigen.head(i).prod();

  int leftOver = flatIndex;
  for (int i = 2; i >= 1; --i) {
    index.at(i) = leftOver / prod_coeff_per_dim(i);
    leftOver %= prod_coeff_per_dim(i);
  }
  index.at(0) = leftOver;

  return index;
}

std::vector<int> getNeighbourIndices(std::vector<int> dim, int index, bool full26Connected) {
  int index3D[3];
  index3D[0] = flatToThreeDimensionalIndex(index, dim)[0];
  index3D[1] = flatToThreeDimensionalIndex(index, dim)[1];
  index3D[2] = flatToThreeDimensionalIndex(index, dim)[2];

  std::vector<int> neighbours;
  if (!full26Connected) {
    if (index3D[0] - 1 > 0) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0] - 1, index3D[1], index3D[2]}, dim));
    }
    if (index3D[0] + 1 < dim[0]) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0] + 1, index3D[1], index3D[2]}, dim));
    }
    if (index3D[1] - 1 > 0) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0], index3D[1] - 1, index3D[2]}, dim));
    }
    if (index3D[1] + 1 < dim[1]) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0], index3D[1] + 1, index3D[2]}, dim));
    }
    if (index3D[2] - 1 > 0) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0], index3D[1], index3D[2] - 1}, dim));
    }
    if (index3D[2] + 1 < dim[2]) {
      neighbours.push_back(threeDimensionalToFlatIndex({index3D[0], index3D[1], index3D[2] + 1}, dim));
    }
  } else {
    for (int i = index3D[0] - 1; i < index3D[0] + 2; i++) {
      if (i < 0 || i > dim[0]) {
        continue;
      }
      for (int j = index3D[1] - 1; j < index3D[1] + 2; j++) {
        if (j < 0 || j > dim[1]) {
          continue;
        }
        for (int k = index3D[2] - 1; k < index3D[2] + 2; k++) {
          if (k < 0 || k > dim[2]) {
            continue;
          }
          if (i == index3D[0] && j == index3D[1] && k == index3D[2]) {
            continue;
          }
          std::vector<int> neighbourIndex{i, j, k};
          neighbours.push_back(threeDimensionalToFlatIndex(neighbourIndex, dim));
        }
      }
    }
  }
  return neighbours;
}

Eigen::VectorXf filterVolumeInsides(const Eigen::VectorXf &volume, std::vector<int> dims, bool full26Connected) {
  Eigen::VectorXf volumeCopy = volume;
  // std::copy(volume.data(), volume.data() + volume.size(), volumeCopy);

  int num_of_points = 0;
  int num_of_filtered_inner_points = 0;
  for (int i = 0; i < volume.size(); i++) {
    if (volume[i] < 0.1) {
      continue;
    }
    num_of_points++;
    std::vector<int> neighbours = getNeighbourIndices(dims, i, full26Connected);
    if (neighbours.size() < (full26Connected ? 26 : 6)) {
      continue;
    }
    bool innerPixel = true;
    for (int j = 0; j < neighbours.size(); j++) {
      if (volume[neighbours[j]] < 0.1) {
        innerPixel = false;
        break;
      }
    }
    if (innerPixel) {
      volumeCopy[i] = 0.0;
      num_of_filtered_inner_points++;
    }
  }
  ROS_DEBUG_STREAM("Filtered points: " + std::to_string(num_of_filtered_inner_points) + " / " +
                   std::to_string(num_of_points));
  return volumeCopy;
}

} // namespace mesh_utils