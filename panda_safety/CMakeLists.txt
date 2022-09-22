cmake_minimum_required(VERSION 2.8.3)
project(panda_safety)

if(SAFETY)
  find_package(catkin REQUIRED
               COMPONENTS roscpp
                          std_msgs
                          tf
                          tf_conversions
                          interactive_markers
                          moveit_core
                          moveit_visual_tools
                          moveit_ros_planning
                          moveit_ros_planning_interface
                          genmsg
                          actionlib_msgs
                          actionlib
                          image_geometry
                          Eigen3
                          controller_manager
                          panda_msgs
                          panda_utils
               REQUIRED
               moveit_ros_planning_interface
               message_filters
               REQUIRED)

  find_package(OpenCL)

  find_package(VTK
             COMPONENTS CommonCore
                        CommonDataModel
                        IOGeometry
                        IOImage
                        ImagingStencil
                        vtkFiltersCore
                        vtkFiltersSources
             REQUIRED)

  catkin_package(INCLUDE_DIRS
                 include
                 CATKIN_DEPENDS
                 panda_utils
                 panda_msgs
                 moveit_core
                 moveit_visual_tools
                 moveit_ros_planning_interface
                 interactive_markers
                 image_geometry
                 DEPENDS
                 Eigen3)

  add_executable(collision_detector src/CollisionDetector.cpp
                                    src/opencl_utils.cpp)
  add_executable(sphere_list_visualizer src/SphereListVisualizer.cpp)

  add_dependencies(collision_detector panda_msgs_generate_messages_cpp)
  add_dependencies(sphere_list_visualizer panda_msgs_generate_messages_cpp)

  target_link_libraries(collision_detector
                        ${catkin_LIBRARIES}
                        ${BOOST_LIBRARIES}
                        boost_timer
                        ${OpenCL_LIBRARIES})
  target_link_libraries(sphere_list_visualizer ${catkin_LIBRARIES})

  target_include_directories(collision_detector
                             PUBLIC include
                                    ${catkin_INCLUDE_DIRS}
                                    ${Eigen_INCLUDE_DIRS}
                                    ${OpenCL_INCLUDE_DIRS})
  target_include_directories(sphere_list_visualizer PUBLIC include)

  add_executable(mesh_reader src/MeshReader.cpp)
  add_dependencies(mesh_reader panda_msgs_generate_messages_cpp)
  target_link_libraries(mesh_reader
                        ${catkin_LIBRARIES}
                        ${VTK_LIBRARIES})
  target_include_directories(mesh_reader PUBLIC include)

else()
  message("Could not find OpenCL. Not Building panda_safety")
endif()
