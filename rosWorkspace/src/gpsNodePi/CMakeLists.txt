cmake_minimum_required(VERSION 3.0.2)
project(gps_node_pi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(sensorMonitor src/gpsNodePi.cpp)
target_link_libraries(sensorMonitor ${catkin_LIBRARIES})
