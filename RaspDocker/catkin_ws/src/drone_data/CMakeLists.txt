cmake_minimum_required(VERSION 3.0.2)
project(drone_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_publisher src/data_publisher.cpp)
target_link_libraries(data_publisher ${catkin_LIBRARIES})