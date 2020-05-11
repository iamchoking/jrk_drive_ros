cmake_minimum_required(VERSION 2.8.3)
project(jrk_drive_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)


catkin_package(
LIBRARIES jrk_drive_ros
CATKIN_DEPENDS std_msgs geometry_msgs rospy
)


include_directories(${catkin_INCLUDE_DIRS})
