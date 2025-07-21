
cmake_minimum_required(VERSION 3.0.2)
project(sensor_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_monitor src/sensor_monitor.cpp)
target_link_libraries(sensor_monitor ${catkin_LIBRARIES})
