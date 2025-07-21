cmake_minimum_required(VERSION 3.0.2)
project(gps_node_pi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_node src/gps_node_pi.cpp)
target_link_libraries(gps_node ${catkin_LIBRARIES})
