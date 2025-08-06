cmake_minimum_required(VERSION 3.0.2)
project(ros_to_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_processor src/data_processor.cpp)
target_link_libraries(data_processor ${catkin_LIBRARIES})
