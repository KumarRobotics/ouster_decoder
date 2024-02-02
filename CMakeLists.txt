cmake_minimum_required(VERSION 3.0.2)
project(ouster_decoder)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  ouster_ros
  )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ouster_decoder2
#  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(decoder_node src/decoder.cpp src/decoder_node.cpp src/lidar.cpp)
target_link_libraries(decoder_node PRIVATE ${catkin_LIBRARIES})

add_executable(driver_node src/driver.cpp src/driver_node.cpp)
target_link_libraries(driver_node PRIVATE ${catkin_LIBRARIES})

