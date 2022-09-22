#include <fstream>
#include <panda_safety/MeshReader.h>

namespace nodes {

using namespace constants;

MeshReader::MeshReader(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
    : node_handle(node_handle), private_node_handle(private_node_handle) {
  this->init();
}

void MeshReader::init() {
  read_mesh_server = node_handle.advertiseService(service_endpoints::READ_MESH, &MeshReader::readMeshCallback, this);
}

bool MeshReader::readMeshCallback(panda_msgs::ReadMesh::Request &req, panda_msgs::ReadMesh::Response &res) {
  ROS_INFO_STREAM("Reading: " << req.volume_read_path.c_str());
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
  reader->SetFileName(req.volume_read_path.c_str());
  reader->Update();

  vtkSmartPointer<vtkPolyData> pd_big = reader->GetOutput();

  ROS_INFO_STREAM("Input cube has " << pd_big->GetNumberOfPoints() << " vertices.");

  vtkNew<vtkCleanPolyData> cleanPolyData;
  cleanPolyData->SetInputData(pd_big);
  cleanPolyData->Update();

  vtkSmartPointer<vtkPolyData> pd = cleanPolyData->GetOutput();

  ROS_INFO_STREAM("Cleaned cube has " << pd->GetNumberOfPoints() << " vertices.");

  vtkSmartPointer<vtkImageData> whiteImage = vtkSmartPointer<vtkImageData>::New();
  double bounds[6];
  pd->GetBounds(bounds);

  double spacing_arr[3] = {req.spacing.at(0), req.spacing.at(1), req.spacing.at(2)};
  whiteImage->SetSpacing(spacing_arr);

  // compute dimensions
  int dim[3];
  for (int i = 0; i < 3; i++) {
    dim[i] = static_cast<int>(ceil((bounds[i * 2 + 1] - bounds[i * 2]) / spacing_arr[i]));
  }
  whiteImage->SetDimensions(dim);
  whiteImage->SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1);

  ROS_INFO_STREAM("bounds");
  ROS_INFO_STREAM(bounds[0] << " " << bounds[1]);
  ROS_INFO_STREAM(bounds[2] << " " << bounds[3]);
  ROS_INFO_STREAM(bounds[4] << " " << bounds[5]);
  double origin[3];
  origin[0] = bounds[0] + spacing_arr[0] / 2;
  origin[1] = bounds[2] + spacing_arr[1] / 2;
  origin[2] = bounds[4] + spacing_arr[2] / 2;
  whiteImage->SetOrigin(origin);
  whiteImage->AllocateScalars(VTK_FLOAT, 1);

  // fill the image with foreground voxels:
  const float inval = req.absorption;
  float outval = 0.0;
  vtkIdType count = whiteImage->GetNumberOfPoints();
  auto scalars_tmp = whiteImage->GetPointData()->GetScalars();
  // #pragma omp parallel for
  for (vtkIdType i = 0; i < count; ++i) {
    scalars_tmp->SetTuple1(i, inval);
  }

  // polygonal data --> image stencil:
  vtkSmartPointer<vtkPolyDataToImageStencil> pol2stenc = vtkSmartPointer<vtkPolyDataToImageStencil>::New();
  pol2stenc->SetInputData(pd);
  pol2stenc->SetOutputOrigin(origin);
  pol2stenc->SetOutputSpacing(spacing_arr);
  pol2stenc->SetOutputWholeExtent(whiteImage->GetExtent());
  pol2stenc->Update();

  // cut the corresponding white image and set the background:
  vtkSmartPointer<vtkImageStencil> imgstenc = vtkSmartPointer<vtkImageStencil>::New();
  imgstenc->SetInputData(whiteImage);
  imgstenc->SetStencilConnection(pol2stenc->GetOutputPort());
  imgstenc->ReverseStencilOff();
  imgstenc->SetBackgroundValue(outval);
  imgstenc->Update();

  auto img_rendered = imgstenc->GetOutput();

  int *dims = img_rendered->GetDimensions();

  size_t volume_size = dims[0] * dims[1] * dims[2];
  ROS_INFO_STREAM("volume dimensions vtk: " << dims[0] << " " << dims[1] << " " << dims[2]);
  ROS_INFO_STREAM("volume size: " << volume_size);
  try {
    ROS_INFO_STREAM("img_rendered num points: " << img_rendered->GetNumberOfPoints());
  } catch (std::exception &e) { ROS_ERROR_STREAM(e.what()); }
  typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, 1> VectorXuc;
  Eigen::VectorXf eigen_volume(Eigen::Map<Eigen::VectorXf>(
      static_cast<float *>(img_rendered->GetScalarPointer(0, 0, 0)), img_rendered->GetNumberOfPoints()));

  auto volume =
      std::vector<float>(eigen_volume.data(), eigen_volume.data() + eigen_volume.rows() * eigen_volume.cols());

  if (req.return_volume) {
    res.volume =
        std::vector<float>(eigen_volume.data(), eigen_volume.data() + eigen_volume.rows() * eigen_volume.cols());
  } else {
    std::ofstream os(req.volume_write_path.c_str(), ios::binary);

    int size1 = volume.size();
    // os.write((const char *)&size1, 4); // why did we add this actually?
    os.write((const char *)&volume[0], size1 * sizeof(float));
    os.close();
  }

  std::vector<int> dimensions = {dims[0], dims[1], dims[2]};
  res.dimensions = dimensions;

  ROS_INFO_STREAM("res volume size: " << res.volume.size());
  geometry_msgs::Point origin_point;
  origin_point.x = origin[0];
  origin_point.y = origin[1];
  origin_point.z = origin[2];
  res.origin = origin_point;
  return true;
}

} // namespace nodes

int main(int argc, char **argv) {
  ros::init(argc, argv, constants::nodes::MESH_READER);

  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");

  nodes::MeshReader node(node_handle, private_node_handle);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}