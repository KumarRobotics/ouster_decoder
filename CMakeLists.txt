cmake_minimum_required(VERSION 3.15)
project(ouster_decoder)

set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport pcl_ros
                                        pcl_conversions sensor_msgs ouster_ros)

catkin_package()

include(CompilerWarnings)
add_library(${PROJECT_NAME}_warnings INTERFACE)
enable_warnings(${PROJECT_NAME}_warnings)

add_executable(ouster_decoder src/decoder.cpp)
target_include_directories(ouster_decoder PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster_decoder PUBLIC ${catkin_LIBRARIES})

add_executable(ouster_driver src/driver.cpp)
target_include_directories(ouster_driver PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster_driver PUBLIC ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}_example src/example.cpp)
# target_include_directories(${PROJECT_NAME}_example PUBLIC
# ${catkin_INCLUDE_DIRS}) target_link_libraries(${PROJECT_NAME}_example PUBLIC
# ${catkin_LIBRARIES} TBB::tbb Boost::timer)
