cmake_minimum_required(VERSION 2.8.3)
project(panda_utils)
find_package(catkin REQUIRED
             COMPONENTS roscpp
                        std_msgs
                        tf
                        tf_conversions
                        interactive_markers
                        genmsg
                        Eigen3
                        geometry_msgs
                        panda_msgs
                        eigen_conversions
                        moveit_msgs
                        Eigen3
             REQUIRED
             moveit_ros_planning_interface
             message_filters
             CONFIG)

message("OpenMesh user-defined prefix: ${OpenMesh_PREFIX}")
find_package(OpenMesh REQUIRED)
set(OpenMesh_INCLUDE_DIRS ${OpenMesh_PREFIX}/include)
set(OPENMESH_CORE_LIBRARY ${OpenMesh_PREFIX}/lib/libOpenMeshCore.so)
set(OPENMESH_TOOLS_LIBRARY ${OpenMesh_PREFIX}/lib/libOpenMeshTools.so)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}_lib
               CATKIN_DEPENDS
               moveit_msgs
               interactive_markers
               DEPENDS
               Eigen3)

set(LIB_SOURCES
    src/mesh_utils.cpp src/ros_utils.cpp)
add_library(${PROJECT_NAME}_lib ${LIB_SOURCES})

add_dependencies(${PROJECT_NAME}_lib panda_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_lib
                      ${catkin_LIBRARIES}
                      ${BOOST_LIBRARIES}
                      ${OPENMESH_TOOLS_LIBRARY}
                      ${OPENMESH_CORE_LIBRARY})

target_include_directories(${PROJECT_NAME}_lib
                           PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OpenMesh_INCLUDE_DIRS})
