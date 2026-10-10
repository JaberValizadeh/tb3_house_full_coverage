cmake_minimum_required(VERSION 3.0.2)
project(tb3_house_full_coverage)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/coverage_node
  scripts/coordinator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
