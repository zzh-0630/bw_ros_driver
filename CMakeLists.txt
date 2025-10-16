cmake_minimum_required(VERSION 3.0.2)
project(bw_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  tf2
  tf2_geometry_msgs 
)

catkin_package(

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(serial_port src/serial_port.cpp)
set_target_properties(serial_port PROPERTIES CXX_STANDARD 14)

add_library(bws_parser src/bws_parser.cpp)
set_target_properties(bws_parser PROPERTIES CXX_STANDARD 14)


add_executable(bw_node src/bw_node.cpp)

target_link_libraries(bw_node 
  bws_parser
  serial_port
  ${catkin_LIBRARIES}
)
