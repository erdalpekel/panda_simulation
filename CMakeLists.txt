cmake_minimum_required(VERSION 2.8.3)
project(panda_simulation)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other
# catkin packages
find_package(catkin REQUIRED
             COMPONENTS controller_manager
                        effort_controllers
                        gazebo_ros
                        joint_state_controller
                        joint_state_publisher
                        joint_trajectory_controller
                        robot_state_publisher
                        roscpp
                        std_msgs
                        tf
                        xacro
                        moveit_core
                        moveit_visual_tools
                        moveit_ros_planning
                        moveit_ros_planning_interface
                        controller_interface
                        hardware_interface)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

# ################################################################################################################################
# catkin specific configuration ##
# ################################################################################################################################
# The catkin_package macro generates cmake config files for your package Declare things to be passed to dependent projects
catkin_package(CATKIN_DEPENDS
               moveit_core
               moveit_visual_tools
               moveit_ros_planning_interface
               controller_interface
               hardware_interface
               pluginlib
               DEPENDS
               system_lib)

# ################################################################################################################################
# Build ##
# ################################################################################################################################

# Specify additional locations of header files Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(robot_control_node src/robot_control_node.cpp)
add_executable(box_publisher_node src/box_publisher.cpp)
add_executable(robot_state_initializer_node src/robot_state_initializer.cpp)

target_link_libraries(robot_control_node ${catkin_LIBRARIES} boost_filesystem ${JSONCPP_LIBRARIES})
target_link_libraries(box_publisher_node ${catkin_LIBRARIES})
target_link_libraries(robot_state_initializer_node ${catkin_LIBRARIES})

# add custom controller as library
add_library(${PROJECT_NAME}_controllers_lib src/joint_position_controller.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_controllers_lib ${catkin_LIBRARIES})
