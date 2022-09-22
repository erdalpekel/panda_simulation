#pragma once

#include <map>
#include <string>
#include <vector>

namespace constants {

static const std::string NAMESPACE = "/panda_simulation";
static const std::string SIMULATION = NAMESPACE + "/simulation";
static const std::string EXPERIMENT = NAMESPACE + "/experiment";
static const std::string NUM_THREADS_PER_NODE = NAMESPACE + "/num_threads_per_node";

static const std::string IMAGE_PLANE_FRAME_SUFFIX = NAMESPACE + "/image_plane_frame_suffix";
static const std::string CAMERA_LINK_SUFFIX = NAMESPACE + "/camera_link_suffix";
static const std::string CAMERA_INFO_SUFFIX = NAMESPACE + "/camera_info_suffix";
static const std::string CAMERA_IMAGE_SUFFIX = NAMESPACE + "/camera_image_suffix";
static const std::string CAMERA_DEPTH_SUFFIX = NAMESPACE + "/camera_depth_suffix";
static const std::string CAMERA_CENTER_FRAME_SUFFIX = NAMESPACE + "/camera_center_frame_suffix";
static const std::string DETECTOR_CAMERA_NAME = NAMESPACE + "/detector_camera_name";
static const std::string CAMERA_GOAL_DISTANCE = NAMESPACE + "/camera_goal_distance";
static const std::string COLLISION_ENV = NAMESPACE + "/collision_env";

static const std::string TRAJECTORY_SUB_GOAL_TOPIC = NAMESPACE + "/trajectory_sub_goal_topic";
static const std::string TRAJECTORY_SUB_RESULT_TOPIC = NAMESPACE + "/trajectory_sub_result_topic";
static const std::string JOINT_POSITION_COMMAND_TOPIC = NAMESPACE + "/joint_position_command_topic";
static const std::string CONTROLLER_MANAGER_SWITCH_TOPIC = NAMESPACE + "/controller_manager_switch_topic";

namespace collision_detection {
static const std::string SUBSPACE("/collision_detection");
static const std::string OPENCL_KERNEL_FAST_DETECTION_PATH =
    NAMESPACE + SUBSPACE + "/opencl_kernel_fast_detection_path";
static const std::string OPENCL_KERNEL_VERIFICATION_PATH = NAMESPACE + SUBSPACE + "/opencl_kernel_verification_path";
static const std::string OPENCL_KERNEL_DETECTION_PATH = NAMESPACE + SUBSPACE + "/opencl_kernel_detection_path";
static const std::string OPENCL_KERNEL_MODE = NAMESPACE + SUBSPACE + "/opencl_kernel_mode";
static const std::string OPENCL_VERIFICATION_KERNEL_OPTIONS =
    NAMESPACE + SUBSPACE + "/opencl_verification_kernel_collected_options";
static const std::string OPENCL_DETECTION_KERNEL_OPTIONS =
    NAMESPACE + SUBSPACE + "/opencl_detection_kernel_collected_options";
static const std::string OPENCL_DEFAULT_DEVICE = NAMESPACE + SUBSPACE + "/opencl_default_device";
static const std::string OPENCL_DEVICE = NAMESPACE + SUBSPACE + "/opencl_device";
static const std::string OPENCL_PLATFORM = NAMESPACE + SUBSPACE + "/opencl_platform";
static const std::string MESH_VOXELATION_SPACING = NAMESPACE + SUBSPACE + "/mesh_voxelation_spacing";
static const std::string OPENCL_DEVICE_PRIVATE = "opencl_device";
static const std::string REFERENCE_LINK_VTK = NAMESPACE + SUBSPACE + "/reference_link_vtk";
} // namespace collision_detection

namespace max_box_pcl {

static const std::string SUBSPACE("/max_box_pcl");
static const std::string MAX_BOX_PCL_MAX = NAMESPACE + SUBSPACE + "/max";
static const std::string MAX_BOX_PCL_MIN = NAMESPACE + SUBSPACE + "/min";

} // namespace max_box_pcl

namespace robot_control {
static const std::string SUBSPACE("/robot_control");
static const std::string PANDA_ARM_TO_HAND_OFFSET = NAMESPACE + SUBSPACE + "/panda_arm_to_hand_offset";
static const std::string PANDA_ARM_VELOCITY_SCALING = NAMESPACE + SUBSPACE + "/panda_arm_velocity_scaling";
static const std::string PANDA_ARM_ACCELERATION_SCALING = NAMESPACE + SUBSPACE + "/panda_arm_acceleration_scaling";
static const std::string ATTACHED_OBJECT_LINK = NAMESPACE + SUBSPACE + "/attached_object_link";
static const std::string GOAL_ORIENTATION_TOLERANCE = NAMESPACE + SUBSPACE + "/goal_orientation_tolerance";
static const std::string GOAL_POSITION_TOLERANCE = NAMESPACE + SUBSPACE + "/goal_position_tolerance";
static const std::string GOAL_JOINT_TOLERANCE = NAMESPACE + SUBSPACE + "/goal_joint_tolerance";
static const std::string DETECTOR_EXPOSURE_TIME = NAMESPACE + SUBSPACE + "/detector_exposure_time";
static const std::string POSE_REFERENCE_FRAME = NAMESPACE + SUBSPACE + "/pose_reference_frame";
static const std::string GRIPPER_HEIGHT = NAMESPACE + SUBSPACE + "/gripper_height";
} // namespace robot_control

static const float CAMERA_ROTATION_ERROR_THRESHOLD = 0.1;
static const float DISTANCE_NORM_TO_CAMERA = 1.0f;
static const float PANDA_JOINT8_OFFSET = 0.107f;

static const std::vector<double> PANDA_READY_STATE_WITH_FINGERS{0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.02, 0.02};
static const std::vector<double> PANDA_READY_STATE{0, -0.785, 0, -2.356, 0, 1.571, 0.785};

static const std::vector<int> COLLISION_SPACE_DIMENSIONS{512, 512, 512};

float PI = 2 * acos(0.0);
std::vector<std::vector<float>> DENAVIT_HARTENBERG_PARAMETERS({{0.0, 0.333, 0.0},
                                                               {0.0, 0.0, -PI / 2},
                                                               {0.0, 0.316, PI / 2},
                                                               {0.0825, 0.0, PI / 2},
                                                               {-0.0825, 0.384, -PI / 2},
                                                               {0.0, 0.0, PI / 2},
                                                               {0.088, 0.0, PI / 2},
                                                               {0.0, 0.107, 0}});

// Trajectory types
std::vector<std::string> TRAJECTORY_TYPES = {"circular", "spherical"};

static const std::string PANDA_SAFETY_PKG("panda_safety");
static const std::string PANDA_ASSETS("/assets");
static const std::string FRANKA_MESH_REL_PATH("/visual_obj_blender/");

static const std::map<std::string, std::string> PANDA_FILE_TO_LINK_MAPPING{
    {"link0.obj", "panda_link0"}, {"link1.obj", "panda_link1"}, {"link2.obj", "panda_link2"},
    {"link3.obj", "panda_link3"}, {"link4.obj", "panda_link4"}, {"link5.obj", "panda_link5"},
    {"link6.obj", "panda_link6"}, {"link7.obj", "panda_link7"}, {"hand.obj", "panda_hand"}};
std::vector<std::string> panda_links{"panda_link1", "panda_link2", "panda_link3", "panda_link4",
                                     "panda_link5", "panda_link6", "panda_link7"};
std::vector<std::string> panda_links_all{"panda_link1", "panda_link2", "panda_link3", "panda_link4",
                                         "panda_link5", "panda_link6", "panda_link7", "panda_link8"};
std::vector<std::string> panda_links_hand{"panda_link1", "panda_link2", "panda_link3", "panda_link4",
                                          "panda_link5", "panda_link6", "panda_link7", "panda_hand"};
std::vector<std::string> panda_joints{"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                      "panda_joint5", "panda_joint6", "panda_joint7"};
std::vector<std::string> panda_mesh_files_stl{"link0.stl", "link1.stl", "link2.stl", "link3.stl", "link4.stl",
                                              "link5.stl", "link6.stl", "link7.stl", "hand.stl"};
std::vector<std::string> panda_mesh_files{"link0.obj", "link1.obj", "link2.obj", "link3.obj", "link4.obj",
                                          "link5.obj", "link6.obj", "link7.obj", "hand.obj"};

static const std::string PLANNING_GROUP_ARM_HAND = "panda_arm_hand";
static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const std::string PLANNING_GROUP_HAND = "hand";
static const std::string PANDA_EE_PARENT_LINK = "panda_link8";
static const std::string PANDA_EE_LINK = "panda_hand";
static const std::vector<std::string> PANDA_FINGER_LINKS{"panda_leftfinger", "panda_rightfinger"};
static const std::string WORLD_LINK = "world";
static const std::string VISUALIZE = "visualize";
static const std::string ROBOT_BASE_LINK = "panda_link0";
static const std::vector<std::string> PANDA_IGNORE_LINKS{"panda_hand", "panda_leftfinger", "panda_rightfinger"};
static const std::string CAMERA_ID = "camera";

namespace service_endpoints {

static const std::string ROBOT_STOP("/robot_control_stop");
static const std::string READ_MESH("/read_mesh");
static const std::string SPHERE_LIST_DISPLAY("/sphere_list_display_");
} // namespace service_endpoints

namespace nodes {

static const std::string CALIBRATION_BOX_PUBLISHER("calibration_box_publisher_node");
static const std::string ROBOT_CONTROL("robot_control_node");
static const std::string COLLISION_DETECTOR("collision_detector_node");
static const std::string COLLISION_VISUALIZER("collision_visualizer_node");
static const std::string MESH_READER("mesh_reader");

} // namespace nodes

namespace visualization {

static const std::string SUBSPACE("/visualization");

static const std::string MARKER_DEPTH_COLLISION = NAMESPACE + SUBSPACE + "/marker_depth_collision";

} // namespace visualization

} // namespace constants
