cmake_minimum_required(VERSION 2.8.3)
project(map_odom_corrector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  tf2_ros
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)