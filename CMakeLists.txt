cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CompilerWarnings)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  ouster_ros
  )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(decoder_node src/decoder.cpp src/decoder_node.cpp src/lidar.cpp)
target_link_libraries(decoder_node PRIVATE ${catkin_LIBRARIES})
enable_warnings(decoder_node)

add_executable(driver_node src/driver.cpp src/driver_node.cpp)
target_link_libraries(driver_node PRIVATE ${catkin_LIBRARIES})
enable_warnings(driver_node)
