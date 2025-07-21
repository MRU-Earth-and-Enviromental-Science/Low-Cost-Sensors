cmake_minimum_required(VERSION 3.0.2)
project(rosToRaw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dataProcessor src/dataProcessor.cpp)
target_link_libraries(dataProcessor ${catkin_LIBRARIES})
