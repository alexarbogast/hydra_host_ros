cmake_minimum_required(VERSION 3.0.2)
project(hydra_moveit_planning)

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

include_directories(${catkin_INCLUDE_DIRS})

add_executable(hydra_demo src/hydra_demo.cpp)
target_link_libraries(hydra_demo ${catkin_LIBRARIES})