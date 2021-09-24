cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CompilerWarnings)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport pcl_ros
                                        pcl_conversions sensor_msgs ouster_ros)
catkin_package()

add_executable(ouster_decoder src/lidar.cpp src/decoder.cpp)
target_include_directories(ouster_decoder PRIVATE src ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster_decoder PRIVATE ${catkin_LIBRARIES})
enable_warnings(ouster_decoder)

add_executable(ouster_driver src/driver.cpp)
target_include_directories(ouster_driver PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster_driver PRIVATE ${catkin_LIBRARIES})
