cmake_minimum_required(VERSION 3.0.2)
project(bw_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs 
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serial_port src/serial_port.cpp)
set_target_properties(serial_port PROPERTIES CXX_STANDARD 14)

add_library(standard_parser src/standard_parser.cpp)
set_target_properties(standard_parser PROPERTIES CXX_STANDARD 14)

add_library(nova_parser src/nova_parser.cpp)
set_target_properties(nova_parser PROPERTIES CXX_STANDARD 14)

add_library(can_port src/can_port.cpp)
set_target_properties(can_port PROPERTIES CXX_STANDARD 14)

add_executable(bw_node_nova src/bw_node_nova.cpp)
add_executable(bw_node_standard src/bw_node_standard.cpp)

add_executable(bw_node_can src/bw_node_can.cpp)
set_target_properties(bw_node_can PROPERTIES CXX_STANDARD 14)

target_link_libraries(bw_node_nova
  standard_parser
  serial_port
  nova_parser  
  ${catkin_LIBRARIES}
)
target_link_libraries(bw_node_standard
  standard_parser
  serial_port
  nova_parser  
  ${catkin_LIBRARIES}
)

target_link_libraries(bw_node_can 
  can_port 
  ${catkin_LIBRARIES}
)
