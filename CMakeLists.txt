cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             image_transport
             pcl_ros
             pcl_conversions
             dynamic_reconfigure
             diagnostic_updater
             ouster_ros)

catkin_package(INCLUDE_DIRS include)

include(CompilerWarnings)
add_library(${PROJECT_NANE}_warnings INTERFACE)
set_project_warnings(${PROJECT_NAME}_warnings)

include_directories(src)
add_subdirectory(src)
