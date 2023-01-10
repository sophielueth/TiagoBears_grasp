cmake_minimum_required(VERSION 2.8.3)
project(TiagoBears_grasp)
find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  control_msgs
  geometry_msgs
)
catkin_package()