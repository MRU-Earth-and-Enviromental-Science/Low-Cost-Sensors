cmake_minimum_required(VERSION 3.0.2)
project(dataToRaw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_packages()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dataToRaw src/dataToRaw.cpp)
target_link_libraries(dataToRaw ${catkin_LIBRARIES})
