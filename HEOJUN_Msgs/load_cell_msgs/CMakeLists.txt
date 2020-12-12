cmake_minimum_required(VERSION 3.0.2)
project(load_cell_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  LoadCellForce.msg
  LoadCellForceArray.msg
  LoadCellForceVector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
