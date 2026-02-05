cmake_minimum_required(VERSION 3.0.2)
project(image_preprocess)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
  FILES
  PreprocessStats.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(PROGRAMS
  scripts/preprocess_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
