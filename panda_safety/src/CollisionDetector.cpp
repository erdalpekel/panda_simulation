#include <panda_safety/CollisionDetector.h>

using namespace constants;

CollisionDetector::CollisionDetector(const ros::NodeHandle &node_handle, const ros::NodeHandle &private_node_handle)
    : node_handle(node_handle), private_node_handle(private_node_handle) {
  this->init();
}

void CollisionDetector::init() {
  // load config values
  try {
    readConfigParameter<std::string>(node_handle, CAMERA_DEPTH_SUFFIX, camera_depth_topic_suffix);
    readConfigParameter<std::string>(node_handle, CAMERA_CENTER_FRAME_SUFFIX, camera_center_frame_suffix);
    readConfigParameter<std::string>(node_handle, TRAJECTORY_SUB_GOAL_TOPIC, trajectory_sub_goal_topic);
    readConfigParameter<std::string>(node_handle, TRAJECTORY_SUB_RESULT_TOPIC, trajectory_sub_result_topic);
    readConfigParameter<double>(node_handle, robot_control::PANDA_ARM_TO_HAND_OFFSET, end_effector_z_offset);
    readConfigParameter<bool>(node_handle, SIMULATION, simulation);
    readConfigParameter<std::string>(node_handle, collision_detection::OPENCL_DETECTION_KERNEL_OPTIONS,
                                     opencl_detection_kernel_collected_options);
    readConfigParameter<std::string>(node_handle, collision_detection::OPENCL_VERIFICATION_KERNEL_OPTIONS,
                                     opencl_verification_kernel_collected_options);
    readConfigParameter<int>(node_handle, collision_detection::OPENCL_DEFAULT_DEVICE, opencl_default_device);
    readConfigParameter<std::string>(node_handle, collision_detection::OPENCL_KERNEL_DETECTION_PATH,
                                     opencl_kernel_detection_path);
    readConfigParameter<std::string>(node_handle, collision_detection::OPENCL_KERNEL_VERIFICATION_PATH,
                                     opencl_kernel_verification_path);
    readConfigParameter<int>(node_handle, collision_detection::OPENCL_PLATFORM, opencl_platform);
    readConfigParameter<std::vector<double>>(node_handle, collision_detection::MESH_VOXELATION_SPACING,
                                             mesh_voxelation_spacing);
    readConfigParameter<std::vector<float>>(node_handle, max_box_pcl::MAX_BOX_PCL_MAX, max_box_pcl_max);
    readConfigParameter<std::vector<float>>(node_handle, max_box_pcl::MAX_BOX_PCL_MIN, max_box_pcl_min);
    readConfigParameter<std::string>(node_handle, collision_detection::REFERENCE_LINK_VTK, reference_link_vtk);
    path_panda_safety = ros::package::getPath(constants::PANDA_SAFETY_PKG);
    if (path_panda_safety.length() == 0) {
      throw std::runtime_error(std::string("Could not find package paths"));
    }
  } catch (const std::runtime_error &exception) { ROS_ERROR_STREAM(exception.what()); }

  // the /joint_state topic publishes the fingers at positions 0 and 1 -> begin from index 2
  index_link_start = 2;

  if (!private_node_handle.getParam(CAMERA_ID, camera_id)) {
    throw std::runtime_error(
        "Parameter \"camera\" was not set when starting script! Specify which camera you are starting!");
  }

  if (!private_node_handle.getParam(VISUALIZE, visualize)) {
    visualize = false;
  }
  camera_frame = "/" + camera_id + camera_center_frame_suffix;
  camera_depth_topic = "/" + camera_id + camera_depth_topic_suffix;

  robot_control_stop = node_handle.serviceClient<panda_msgs::RobotStopMsg>(service_endpoints::ROBOT_STOP);
  sphere_list_display =
      node_handle.serviceClient<panda_msgs::SphereListDisplayMsg>(service_endpoints::SPHERE_LIST_DISPLAY + camera_id);
  read_mesh_client = node_handle.serviceClient<panda_msgs::ReadMesh>(service_endpoints::READ_MESH);

  // sleep for tf_listener to prepare in the background. Otherwise the first tf request return error
  ros::Duration(5.0).sleep();

  robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  std::string kinematic_model_frame = kinematic_model->getModelFrame();
  ROS_WARN_STREAM("kinematic state modelframe: " << kinematic_model_frame);
  ros_utils::transformAsMatrix(ros_utils::getTransformToTargetFrame(tf_listener, kinematic_model_frame, camera_frame),
                               transform_depth_camera);
  ROS_INFO_STREAM(kinematic_model_frame << " to depth camera transform:" << std::endl << transform_depth_camera);

  // read and convert mesh file
  std::vector<std::vector<int>> dims;
  std::vector<cl_int> num_vertices;
  std::vector<geometry_msgs::Point> origin;
  num_mesh_vertices = 0;
  for (int i = 0; i < panda_mesh_files.size() - 1; i++) {
    std::string opencl_detection_kernel_options = "";
    opencl_detection_kernel_options +=
        "-D JOINT_" + std::to_string(i) + "_a=" + std::to_string(DENAVIT_HARTENBERG_PARAMETERS[i][0]) + "f ";
    opencl_detection_kernel_options +=
        "-D JOINT_" + std::to_string(i) + "_d=" + std::to_string(DENAVIT_HARTENBERG_PARAMETERS[i][1]) + "f ";
    opencl_detection_kernel_options +=
        "-D JOINT_" + std::to_string(i) + "_alpha=" + std::to_string(DENAVIT_HARTENBERG_PARAMETERS[i][2]) + "f ";
    opencl_detection_kernel_collected_options += opencl_detection_kernel_options;
    ROS_INFO_STREAM(opencl_detection_kernel_options);
  }

  std::vector<Eigen::VectorXf> mesh_volumen;
  std::vector<geometry_msgs::Point> volume_points;

  for (int i = 0; i < panda_mesh_files.size(); i++) {
    Eigen::VectorXf volume;
    std::vector<int> volume_dimensions;
    panda_msgs::ReadMesh srv_read_mesh;
    srv_read_mesh.request.volume_read_path =
        path_panda_safety + constants::PANDA_ASSETS + FRANKA_MESH_REL_PATH + panda_mesh_files[i];
    srv_read_mesh.request.spacing = mesh_voxelation_spacing;
    srv_read_mesh.request.return_volume = true;
    srv_read_mesh.request.absorption = 1.0;
    if (read_mesh_client.call(srv_read_mesh)) {
      ROS_INFO("Read mesh call successful");
      dims.push_back(srv_read_mesh.response.dimensions);
      origin.push_back(srv_read_mesh.response.origin);
      int vol_size_tmp = dims[i].at(0) * dims[i].at(1) * dims[i].at(2);
      auto mesh_full_volume =
          Eigen::VectorXf(Eigen::Map<Eigen::VectorXf>(srv_read_mesh.response.volume.data(), vol_size_tmp));
      bool full26connected = false;
      mesh_volumen.push_back(mesh_utils::filterVolumeInsides(mesh_full_volume, dims[i], full26connected));
    } else {
      ROS_ERROR("Read mesh call not successful");
      return;
    }
  }

  // retrieve transform from vtk reference link to world
  tf::StampedTransform vtk_to_world_tf =
      ros_utils::getTransformToTargetFrame(tf_listener, WORLD_LINK, reference_link_vtk);
  tf::poseTFToEigen(vtk_to_world_tf, vtk_to_world_eigen);
  for (int i = 0; i < mesh_volumen.size(); i++) {
    auto filename = panda_mesh_files.at(i);
    auto frame_id = PANDA_FILE_TO_LINK_MAPPING.find(filename)->second;
    num_vertices.push_back(0);

    Eigen::Isometry3d robot_link_to_world_eigen;
    tf::StampedTransform robot_link_to_world_tf =
        ros_utils::getTransformToTargetFrame(tf_listener, WORLD_LINK, frame_id);
    tf::poseTFToEigen(robot_link_to_world_tf, robot_link_to_world_eigen);
    for (int j = 0; j < mesh_volumen[i].size(); j++) {
      if (mesh_volumen[i][j] > 0.0) {
        num_vertices[i]++;
        num_mesh_vertices++;
        std::vector<int> index3D = mesh_utils::flatToThreeDimensionalIndex(j, dims[i]);
        Eigen::Vector3d newVertex;
        newVertex(0) = origin.at(i).x + index3D.at(0) * mesh_voxelation_spacing.at(0);
        newVertex(1) = origin.at(i).y + index3D.at(1) * mesh_voxelation_spacing.at(1);
        newVertex(2) = origin.at(i).z + index3D.at(2) * mesh_voxelation_spacing.at(2);

        auto point_in_link_frame_eigen = (vtk_to_world_eigen.matrix() * newVertex.homogeneous());

        Eigen::Vector3f vertex_in_link_frame;
        vertex_in_link_frame[0] = point_in_link_frame_eigen.x();
        vertex_in_link_frame[1] = point_in_link_frame_eigen.y();
        vertex_in_link_frame[2] = point_in_link_frame_eigen.z();

        link_vertices.push_back(vertex_in_link_frame);

        // visualize
        auto point_in_world_frame_eigen =
            robot_link_to_world_eigen.matrix() * (vtk_to_world_eigen.matrix() * newVertex.homogeneous());

        geometry_msgs::Point point_in_world_frame;
        point_in_world_frame.x = point_in_world_frame_eigen.x();
        point_in_world_frame.y = point_in_world_frame_eigen.y();
        point_in_world_frame.z = point_in_world_frame_eigen.z();

        volume_points.push_back(point_in_world_frame);
      }
    }
  }
  // num_mesh_vertices = 1168;
  std::string detection_kernel_num_mesh_arg = "-D NUM_MESH_VERTICES=" + std::to_string(num_mesh_vertices) + "";
  opencl_detection_kernel_collected_options += detection_kernel_num_mesh_arg;
  if (visualize) {
    opencl_verification_kernel_collected_options += "-D VISUALIZE";
  }

  try {
    auto platform = opencl_utils::getOpenCLPlatform(opencl_platform);
    auto devices = opencl_utils::getOpenCLDevices(platform);
    context = cl::Context(devices);

    // select which OpenCL device to use
    use_opencl_device = opencl_default_device;
    if (private_node_handle.getParam(collision_detection::OPENCL_DEVICE_PRIVATE, use_opencl_device) &&
        use_opencl_device <= devices.size()) {
      ROS_INFO_STREAM("Will use opencl device ID / NUM: " << use_opencl_device);
    }
    opencl_queue = opencl_utils::getCommandQueue(context, devices[use_opencl_device]);

    size_t workgroup_size = opencl_utils::getWorkgroupSize(devices[use_opencl_device]);
    //  prevent OUT_OF_RESOURCES ERRORS caused by too many registers used in workgroup
    if (workgroup_size > 256) {
      workgroup_size = 256;
    }
    workgroup_range = opencl_utils::getWorkgroupRange(workgroup_size);

    opencl_utils::buildOpenCLProgram(path_panda_safety + opencl_kernel_detection_path, program_detection, context,
                                     &devices, opencl_detection_kernel_collected_options);
    detection_kernel = cl::Kernel(program_detection, "detection_kernel");

    opencl_utils::buildOpenCLProgram(path_panda_safety + opencl_kernel_verification_path, program_verification, context,
                                     &devices, opencl_verification_kernel_collected_options);
    verification_kernel = cl::Kernel(program_verification, "verification_kernel");
  } catch (const std::exception &exception) {
    ROS_ERROR(exception.what());
    // throw exception;
  }

  // OpenCL setup - init buffers
  cl_int status;
  collision_width = COLLISION_SPACE_DIMENSIONS[0];
  collision_height = COLLISION_SPACE_DIMENSIONS[0];
  collision_depth = COLLISION_SPACE_DIMENSIONS[0];
  result_length = visualize ? collision_width * collision_height * collision_depth : 1;

  transforms_buffer =
      cl::Buffer(context, CL_MEM_READ_ONLY, (constants::panda_links.size() + 2) * sizeof(cl_float16), NULL, &status);
  ROS_INFO_STREAM("STATUS transforms_buffer: " << status);
  transform_camera_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, 3 * sizeof(cl_float4), NULL, &status);
  ROS_INFO_STREAM("STATUS transform_camera_buffer: " << status);
  joint_position_trajectory_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, 50 * 7 * sizeof(cl_float), NULL, &status);
  ROS_INFO_STREAM("STATUS joint_position_trajectory_buffer: " << status);
  maximum_reachable_box = (cl_float3){max_box_pcl_max.at(0), max_box_pcl_max.at(1), max_box_pcl_max.at(2)};
  minimum_reachable_box = (cl_float3){max_box_pcl_min.at(0), max_box_pcl_min.at(1), max_box_pcl_min.at(2)};
  dimensions_reachable_box.push_back(maximum_reachable_box.s[0] - minimum_reachable_box.s[0]);
  dimensions_reachable_box.push_back(maximum_reachable_box.s[1] - minimum_reachable_box.s[1]);
  dimensions_reachable_box.push_back(maximum_reachable_box.s[2] - minimum_reachable_box.s[2]);

  // set kernel args - detection kernel
  status = detection_kernel.setArg(1, transforms_buffer);
  ROS_INFO_STREAM("STATUS setArg transforms_buffer: " << status);
  status = detection_kernel.setArg(6, joint_position_trajectory_buffer);
  ROS_INFO_STREAM("STATUS setArg joint_position_trajectory_trajectory_buffer: " << status);
  status = detection_kernel.setArg(2, transform_camera_buffer);
  ROS_INFO_STREAM("STATUS setArg transform_camera_buffer: " << status);
  status = detection_kernel.setArg(7, maximum_reachable_box);
  ROS_INFO_STREAM("STATUS setArg maximum_reachable_box: " << status);
  status = detection_kernel.setArg(8, minimum_reachable_box);
  ROS_INFO_STREAM("STATUS setArg minimum_reachable_box: " << status);

  // set kernel args - verification kernel
  result_kernel_verification_buffer =
      cl::Buffer(context, CL_MEM_WRITE_ONLY, sizeof(bool) * result_length, NULL, &status);
  ROS_DEBUG_STREAM("STATUS result_kernel_verification_buffer: " << status);
  status = verification_kernel.setArg(0, result_kernel_verification_buffer);
  ROS_DEBUG_STREAM("STATUS setArg kernel verification result_kernel_verification_buffer: " << status);

  num_vertices_buffer = cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                   num_vertices.size() * sizeof(cl_int), num_vertices.data(), &status);
  ROS_INFO_STREAM("num vertices size: " << num_vertices.size());
  ROS_INFO_STREAM("STATUS num_vertices buffer: " << status);
  status = detection_kernel.setArg(9, num_vertices_buffer);
  ROS_INFO_STREAM("STATUS setArg num vertices : " << status);

  mesh_buffer = cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, link_vertices.size() * sizeof(cl_float3),
                           link_vertices.data(), &status);
  ROS_INFO_STREAM("link vertices size: " << link_vertices.size());
  ROS_INFO_STREAM("STATUS link_vertices buffer: " << status);
  status = detection_kernel.setArg(5, mesh_buffer);
  ROS_INFO_STREAM("STATUS setArg mesh link : " << status);

  // put the transform in the buffer row-wise as this makes matrix mult easier on the kernel
  transform_camera.push_back((cl_float4){transform_depth_camera.col(0)[0], transform_depth_camera.col(1)[0],
                                         transform_depth_camera.col(2)[0], transform_depth_camera.col(3)[0]});
  transform_camera.push_back((cl_float4){transform_depth_camera.col(0)[1], transform_depth_camera.col(1)[1],
                                         transform_depth_camera.col(2)[1], transform_depth_camera.col(3)[1]});
  transform_camera.push_back((cl_float4){transform_depth_camera.col(0)[2], transform_depth_camera.col(1)[2],
                                         transform_depth_camera.col(2)[2], transform_depth_camera.col(3)[2]});
  opencl_queue.enqueueWriteBuffer(transform_camera_buffer, CL_BLOCKING, 0, transform_camera.size() * sizeof(cl_float4),
                                  transform_camera.data());

  // Receive transform to link0 beforehand as it is statically mounted
  Eigen::Matrix4f transform_panda_link0_tmp;
  ros_utils::transformAsMatrix(ros_utils::getTransformToTargetFrame(tf_listener, kinematic_model_frame, "panda_link0"),
                               transform_panda_link0_tmp);
  transform_panda_link0 = transform_panda_link0_tmp.cast<double>();

  // debug show links
  bool show_at_start = true;
  if (!private_node_handle.getParam("show_at_start", show_at_start)) {
    show_at_start = false;
  }
  if (show_at_start) {
    panda_msgs::SphereListDisplayMsg srv_show_mesh_points;
    srv_show_mesh_points.request.sphere_list = volume_points;
    srv_show_mesh_points.request.frame_id = WORLD_LINK;
    ros::Duration(3.0).sleep();
    if (sphere_list_display.call(srv_show_mesh_points)) {
      ROS_INFO("client call successfull for mesh_points");
    } else {
      ROS_INFO("client call not successfull mesh_points");
    }
  }
  // end debug show link

  this->initDepthResolution();
  this->initResultBufferDevice();
  this->initClImage3dBufferDevice();

  joint_states_sub = node_handle.subscribe("/joint_states", 1, &CollisionDetector::jointStatesCallback, this);
  trajectory_sub = node_handle.subscribe(trajectory_sub_goal_topic, 20, &CollisionDetector::trajectoryCallback, this);
  trajectory_result_sub =
      node_handle.subscribe(trajectory_sub_result_topic, 20, &CollisionDetector::trajectoryResultCallback, this);
  depth_camera_sub = node_handle.subscribe(camera_depth_topic, 1, &CollisionDetector::collisionDetectionCallback, this);
}

void CollisionDetector::initDepthResolution() {
  sensor_msgs::PointCloud2ConstPtr cloud_msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_depth_topic, node_handle);
  if (cloud_msg != nullptr) {
    depth_resolution = cloud_msg->height * cloud_msg->width;

    cl_int status;
    cloud_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, depth_resolution * sizeof(float) * 8, NULL, &status);
    ROS_DEBUG_STREAM("STATUS cloud_buffer: " << status);
    status = detection_kernel.setArg(0, cloud_buffer);
    ROS_DEBUG_STREAM("STATUS setArg cloud_buffer: " << status);
  } else {
    throw std::runtime_error("Could not determine depth camera resolution. Exiting.");
  }
}

void CollisionDetector::initResultBufferDevice() {
  bool *result = new bool[result_length];
  for (int i = 0; i < (result_length); ++i) {
    result[i] = false;
  }
  cl_int status = opencl_queue.enqueueWriteBuffer(result_kernel_verification_buffer, CL_BLOCKING | CL_MEM_COPY_HOST_PTR,
                                                  0, sizeof(bool) * result_length, result);
  ROS_DEBUG_STREAM("STATUS enqueue write buffer resultlernel2Buffer: " << status);
  delete[] result;
}

void CollisionDetector::initClImage3dBufferDevice() {
  cl::ImageFormat format;
  format.image_channel_data_type = CL_UNSIGNED_INT8;
  format.image_channel_order = CL_RGBA;
  cl_int status;
  std::vector<int8_t> host_ptr(4 * collision_depth * collision_height * collision_width, 0);
  clImage = cl::Image3D(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, format, collision_width, collision_height,
                        collision_depth, 0, 0, host_ptr.data(), &status);
  ROS_INFO_STREAM("STATUS 3DImage: " << status);
}

void CollisionDetector::collisionDetectionCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  if (!joint_position_trajectory.empty()) {
    std::string frame_id_current_camera = cloud_msg->header.frame_id;
    ROS_DEBUG_STREAM("cloud_msg frame_id: " << frame_id_current_camera);

    cl_int status;
    auto t_start = std::clock();
    auto t_start_overall = std::clock();

    sensor_msgs::PointCloud2 cloud = *cloud_msg;
    ROS_DEBUG_STREAM("PointCloud has: " << cloud.data.size() << " data points.");
    size_t cloud_offset = cloud.point_step;

    std::vector<cl::Event> event_list;
    cl::Event write_pcl_buffer, write_mesh_buffer, kernel_detection, kernel_verification, read_buffer_result;

    // time lag handling
    std::uint64_t key = cloud_msg->header.stamp.toNSec();
    ROS_INFO_STREAM("Pointcloud timestamp: " << key);
    auto it_tmp = joint_states_buffer.lower_bound(key);
    std::uint64_t tmp_states_key;
    ROS_INFO_STREAM("joint_states timestamp: " << tmp_states_key);
    if (it_tmp != joint_states_buffer.end()) {
      tmp_states_key = it_tmp->first;
    } else {
      tmp_states_key = joint_states_buffer.find(key_deque.back())->first;
      ROS_WARN_STREAM(
          "There is no joint_states with a timestamp suitable to pointcloud timestamp! Skipping this frame");
      return;
    }
    const sensor_msgs::JointState joint_states_current =
        it_tmp != joint_states_buffer.end() ? it_tmp->second : joint_states_buffer.find(key_deque.back())->second;
    setJointStatesArg(joint_states_current);

    auto t_end = std::clock();
    auto t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used for setting joint states: " << t_diff_in_ms
                               << "ms");
    t_start = std::clock();

    // Transfer input data to device.
    status = opencl_queue.enqueueWriteBuffer(cloud_buffer, CL_BLOCKING, 0, depth_resolution * cloud.point_step,
                                             cloud.data.data(), NULL, &write_pcl_buffer);
    ROS_DEBUG_STREAM("status enqueuewrite pointcloud: " << status);

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used for writing pcl buffer: " << t_diff_in_ms
                               << "ms");
    t_start = std::clock();

    event_list.push_back(write_pcl_buffer);
    cl_float3 link_vertices_tmp[num_mesh_vertices];
    for (int i = 0; i < num_mesh_vertices; i++) {
      link_vertices_tmp[i] = (cl_float3){link_vertices[i](0), link_vertices[i](1), link_vertices[i](2)};
    }
    status = opencl_queue.enqueueWriteBuffer(mesh_buffer, CL_BLOCKING, 0, sizeof(cl_float3) * num_mesh_vertices,
                                             &link_vertices_tmp, NULL, &write_mesh_buffer);
    ROS_DEBUG_STREAM("status enqueuewrite mesh: " << status);

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used for writing mesh buffer: " << t_diff_in_ms
                               << "ms");
    t_start = std::clock();

    event_list.push_back(write_mesh_buffer);

    status = detection_kernel.setArg(4, clImage);
    ROS_INFO_STREAM("STATUS setArg OpenCl3DImage: " << status);

    cl::NDRange range(depth_resolution);
    status = opencl_queue.enqueueNDRangeKernel(detection_kernel, cl::NullRange, range, workgroup_range, &event_list,
                                               &kernel_detection);
    ROS_INFO_STREAM("STATUS enqueueNDRangeKernel detection kernel: " << status);

    opencl_queue.finish();

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used for detection kernel: " << t_diff_in_ms
                               << "ms");
    t_start = std::clock();

    status = verification_kernel.setArg(1, clImage);
    ROS_INFO_STREAM("STATUS setArg kernel verification OpenCl3DImage: " << status);
    status = verification_kernel.setArg(0, result_kernel_verification_buffer); // use single bool for release
    ROS_INFO_STREAM("STATUS setArg kernel verification result_kernel_verification_buffer: " << status);

    event_list.push_back(kernel_detection);

    status = opencl_queue.enqueueNDRangeKernel(verification_kernel, cl::NullRange,
                                               cl::NDRange(collision_width, collision_height, collision_depth),
                                               cl::NullRange, &event_list, &kernel_verification);
    ROS_INFO_STREAM("STATUS enqueueNDRangeKernel kernel verification: " << status);
    opencl_queue.finish();

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used for verification kernel: " << t_diff_in_ms
                               << "ms");
    t_start = std::clock();

    event_list.push_back(kernel_verification);

    bool *result_verified = new bool[result_length];
    status = opencl_queue.enqueueReadBuffer(result_kernel_verification_buffer, CL_TRUE, 0, sizeof(bool) * result_length,
                                            result_verified, &event_list, &read_buffer_result);
    ROS_DEBUG_STREAM("STATUS enqueueReadBuffer result_kernel_verification_buffer: " << status);

    event_list.push_back(read_buffer_result);
    cl::WaitForEvents(event_list);

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                               << "CPU time used for reading result from kernel: " << t_diff_in_ms << "ms");
    t_start = std::clock();

    size_t counter_collisions = 0;
    std::vector<geometry_msgs::Point> collision_points;
    for (size_t i = 0; i < result_length; ++i) {
      if (result_verified[i]) {
        ++counter_collisions;
        if (visualize) {
          geometry_msgs::Point point_publish;
          size_t index = i * cloud_offset;
          std::vector<int> indices = mesh_utils::flatToThreeDimensionalIndex(
              i, std::vector<int>{(int)collision_height, (int)collision_width, (int)collision_depth});
          point_publish.x = (((float)indices[0]) / (float)collision_height) * (float)dimensions_reachable_box[0] +
                            minimum_reachable_box.s[0];
          point_publish.y = (((float)indices[1]) / (float)collision_width) * (float)dimensions_reachable_box[1] +
                            minimum_reachable_box.s[1];
          point_publish.z = (((float)indices[2]) / (float)collision_depth) * (float)dimensions_reachable_box[2] +
                            minimum_reachable_box.s[2];
          Eigen::Vector4f point_publish_tmp = transform_depth_camera.inverse() *
                                              Eigen::Vector4f(point_publish.x, point_publish.y, point_publish.z, 1.0);
          point_publish.x = point_publish_tmp[0];
          point_publish.y = point_publish_tmp[1];
          point_publish.z = point_publish_tmp[2];
          collision_points.push_back(point_publish);
        }
      }
    }
    delete[] result_verified;
    ROS_DEBUG_STREAM("collision counter: " << counter_collisions);

    if (counter_collisions > 0) {
      ROS_WARN_STREAM(frame_id_current_camera << " : Collisions detected for the currently executed trajectory: "
                                              << counter_collisions);
      panda_msgs::RobotStopMsg srv;
      if (robot_control_stop.call(srv)) {
        ROS_INFO("client call successfull for STOP");
      } else {
        ROS_INFO("client call not successfull for STOP");
      }
      if (visualize) {
        panda_msgs::SphereListDisplayMsg srv_collision_points;
        srv_collision_points.request.sphere_list = collision_points;
        srv_collision_points.request.frame_id = cloud_msg->header.frame_id;
        if (sphere_list_display.call(srv_collision_points)) {
          ROS_INFO("client call successfull for collision_points_display");
        } else {
          ROS_INFO("client call not successfull collision_points_display");
        }
      }
    } else {
      ROS_INFO_STREAM(frame_id_current_camera << " No collision detected for the currently executed trajectory");
    }

    t_end = std::clock();
    t_diff_in_ms = 1000.0 * (t_end - t_start_overall) / CLOCKS_PER_SEC;
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "CPU time used total: " << t_diff_in_ms << "ms");

    // flush device buffer, otherwise the collision points from previous runs add up in buffer
    initResultBufferDevice();
    initClImage3dBufferDevice();
  }
}

void CollisionDetector::setJointStatesArg(const sensor_msgs::JointState &joint_states_current) {
  std::vector<cl_float16> transforms_flat;
  std::vector<Eigen::Affine3d> transforms;
  std::vector<double> joint_states;
  // we don't check if hand is connected to change array access accordingly because it is always loaded in the
  // simulation first two fields are finger joints. we don't need them for filtering purposes
  for (size_t i = index_link_start; i < joint_states_current.position.size(); ++i) {
    joint_states.push_back(joint_states_current.position[i]);
  }
  kinematic_state->setJointGroupPositions(joint_model_group, joint_states);

  for (auto const &panda_link : constants::panda_links) {
    transforms.push_back(kinematic_state->getGlobalLinkTransform(panda_link));
  }

  cl_int status;
  // Prepare input data.
  // insert link0 transform at the beginning of transform_copy
  transforms.insert(transforms.begin(), transform_panda_link0);
  for (const auto &transf : transforms) {
    auto transf_mat = transf.matrix();
    transforms_flat.push_back((cl_float16){
        transf_mat.row(0)[0], transf_mat.row(0)[1], transf_mat.row(0)[2], transf_mat.row(0)[3], transf_mat.row(1)[0],
        transf_mat.row(1)[1], transf_mat.row(1)[2], transf_mat.row(1)[3], transf_mat.row(2)[0], transf_mat.row(2)[1],
        transf_mat.row(2)[2], transf_mat.row(2)[3], 0.0f, 0.0f, 0.0f, 1.0f});
  }
  auto transform_hand = transforms.back() * Eigen::Translation3d(0.0, 0.0, PANDA_JOINT8_OFFSET);
  auto transf_mat = transform_hand.matrix();
  transforms_flat.push_back((cl_float16){
      transf_mat.row(0)[0], transf_mat.row(0)[1], transf_mat.row(0)[2], transf_mat.row(0)[3], transf_mat.row(1)[0],
      transf_mat.row(1)[1], transf_mat.row(1)[2], transf_mat.row(1)[3], transf_mat.row(2)[0], transf_mat.row(2)[1],
      transf_mat.row(2)[2], transf_mat.row(2)[3], 0.0f, 0.0f, 0.0f, 1.0f});

  cl_int status_write = opencl_queue.enqueueWriteBuffer(
      transforms_buffer, CL_BLOCKING, 0, transforms_flat.size() * sizeof(cl_float16), transforms_flat.data());
  ROS_INFO_STREAM("transforms flat size: " << transforms_flat.size());
  ROS_INFO_STREAM("enqueue write joint states : " << status_write);
}

void CollisionDetector::jointStatesCallback(
    const sensor_msgs::JointState &joint_states_current) { // save all in map and time stamp
  std::size_t size = joint_states_buffer.size();
  if (joint_states_buffer.size() >= 2000) {
    joint_states_buffer.erase(joint_states_buffer.find(key_deque.front()));

    key_deque.pop_front();
  }
  std::uint64_t key = joint_states_current.header.stamp.toNSec();
  const sensor_msgs::JointState joint_states_current_tmp = joint_states_current;
  joint_states_buffer.insert(std::make_pair(key, joint_states_current_tmp));
  key_deque.push_back(key);
}

void CollisionDetector::trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal &joint_trajectory_goal) {
  joint_position_trajectory.clear();
  const size_t trajectory_size = joint_trajectory_goal.goal.trajectory.points.size();
  ROS_INFO_STREAM("trajectory size: " << trajectory_size);

  for (int i = 0; i < trajectory_size; ++i) {
    for (int j = 0; j < joint_trajectory_goal.goal.trajectory.points[i].positions.size(); j++) {
      joint_position_trajectory.push_back(joint_trajectory_goal.goal.trajectory.points[i].positions[j]);
    }
  }
  ROS_DEBUG_STREAM("trajectory vector size: " << joint_position_trajectory.size());

  if (!joint_position_trajectory.empty()) {

    cl_int status = opencl_queue.enqueueWriteBuffer(joint_position_trajectory_buffer, CL_BLOCKING, 0,
                                                    joint_position_trajectory.size() * sizeof(cl_float),
                                                    joint_position_trajectory.data());
    ROS_DEBUG_STREAM("enqueue write joint_position_trajectory buffer: " << status);

    status = detection_kernel.setArg(3, (cl_uint)joint_position_trajectory.size());
    ROS_DEBUG_STREAM("STATUS setArg joint_position_trajectory size: " << status);
  }
}

void CollisionDetector::trajectoryResultCallback(
    const control_msgs::FollowJointTrajectoryActionResult &joint_trajectory_result) {
  ROS_WARN_STREAM("joint_trajectory_result: " << std::endl << joint_trajectory_result);

  switch (joint_trajectory_result.status.status) {
  case control_msgs::FollowJointTrajectoryActionResult::_status_type::ABORTED:
    ROS_WARN_STREAM("trajectory execution ABORTED");
    joint_position_trajectory.clear();
    break;
  case control_msgs::FollowJointTrajectoryActionResult::_status_type::SUCCEEDED:
    ROS_INFO_STREAM("Trajectory execution SUCCEEDED");
    joint_position_trajectory.clear();
    break;
  case control_msgs::FollowJointTrajectoryActionResult::_status_type::ACTIVE:
    ROS_INFO_STREAM("Trajectory is ACTIVE");
    break;
  case control_msgs::FollowJointTrajectoryActionResult::_status_type::PENDING:
    ROS_INFO_STREAM("Trajectory is PENDING");
    break;
  case control_msgs::FollowJointTrajectoryActionResult::_status_type::PREEMPTED:
    ROS_INFO_STREAM("Trajectory execution PREEMPTED");
    joint_position_trajectory.clear();
    break;
  default:
    ROS_WARN_STREAM("Unexpected trajectory result type. Clearing trajectory transforms.");
    joint_position_trajectory.clear();
    break;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, nodes::COLLISION_DETECTOR);
  ros::NodeHandle node_handle("");
  ros::NodeHandle private_node_handle("~");
  CollisionDetector node(node_handle, private_node_handle);

  ros::spin();
  return 0;
}
