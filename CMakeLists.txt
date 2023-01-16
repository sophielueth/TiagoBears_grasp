cmake_minimum_required(VERSION 2.8.3)
project(TiagoBears_grasp)
find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  control_msgs
  geometry_msgs
)
catkin_package()

catkin_install_python(PROGRAMS scripts/test_cubes.py
                               scripts/grasp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()