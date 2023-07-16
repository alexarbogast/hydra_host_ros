cmake_minimum_required(VERSION 3.0.2)
project(za_moveit_planning)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
)

catkin_package(CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
)
