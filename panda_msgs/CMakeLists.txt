cmake_minimum_required(VERSION 2.8.3)
project(panda_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_service_files(FILES
    RobotStopMsg.srv
    SphereListDisplayMsg.srv
    ReadMesh.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
