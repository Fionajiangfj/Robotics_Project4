cmake_minimum_required(VERSION 3.0.2)
project(mapping_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  gmapping
  rviz
  rosbag
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
) 