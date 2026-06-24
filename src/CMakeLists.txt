cmake_minimum_required(VERSION 3.0.2)
project(warthog_simulator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  ros_nodes/surface_detector.py
  ros_nodes/trajectory_runner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  bag_parser
  scripts
  urdf
  world_generation
  worlds_library
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  simulation.launch
  requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
