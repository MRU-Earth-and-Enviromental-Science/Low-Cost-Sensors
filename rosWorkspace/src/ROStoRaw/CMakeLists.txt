cmake_minimum_required(VERSION 3.0.2)
project(ROStoRaw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(data_processor src/dataProcessor.cpp)
target_link_libraries(data_processor ${catkin_LIBRARIES})
