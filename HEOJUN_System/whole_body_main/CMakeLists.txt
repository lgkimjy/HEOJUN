cmake_minimum_required(VERSION 3.0.2)
project(whole_body_main)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  load_cell_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  load_cell_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whole_body_main
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs load_cell_msgs
  #DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(whole_body_main_node src/whole_body_main.cpp)
target_link_libraries(whole_body_main_node ${catkin_LIBRARIES})
add_dependencies(whole_body_main_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})