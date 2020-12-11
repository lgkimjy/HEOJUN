cmake_minimum_required(VERSION 3.0.2)
project(load_cell)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  load_cell_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES load_cell
  CATKIN_DEPENDS roscpp rospy std_msgs load_cell_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(load_cell_node src/load_cell_node.cpp)
add_dependencies(load_cell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_cell_node ${catkin_LIBRARIES})

