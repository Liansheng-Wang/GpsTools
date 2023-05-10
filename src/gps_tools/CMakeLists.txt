cmake_minimum_required(VERSION 3.0.2)
project(gps_tools)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
 # INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(GpsReceiveNode src/GpsSeries.cpp)
target_link_libraries(GpsReceiveNode  ${catkin_LIBRARIES} )
