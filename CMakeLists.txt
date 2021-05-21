cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport pcl_ros
                                        pcl_conversions ouster_ros)

catkin_package()

include(CompilerWarnings)
add_library(${PROJECT_NAME}_warnings INTERFACE)
set_target_warnings(${PROJECT_NAME}_warnings)

add_executable(${PROJECT_NAME} src/decoder.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES}
                                             ${PROJECT_NAME}_warnings)
