cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport pcl_ros
                                        pcl_conversions sensor_msgs ouster_ros)
find_package(GTest REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}_model)
include_directories(include src ${catkin_INCLUDE_DIRS})

include(CompilerWarnings)
add_library(${PROJECT_NAME}_warnings INTERFACE)
set_target_warnings(${PROJECT_NAME}_warnings)

add_library(${PROJECT_NAME}_model src/model.cpp)
target_link_libraries(${PROJECT_NAME}_model PUBLIC ${catkin_LIBRARIES}
                                                   ${PROJECT_NAME}_warnings)
add_executable(${PROJECT_NAME}_model_test src/model_test.cpp)
target_link_libraries(${PROJECT_NAME}_model_test GTest::Main GTest::GTest
                      ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/decoder.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES}
                                                  ${PROJECT_NAME}_model)
