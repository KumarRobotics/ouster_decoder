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

add_library(ouster_decoder_base INTERFACE)
target_include_directories(ouster_decoder_base INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(ouster_decoder_base INTERFACE ${catkin_LIBRARIES})

add_executable(ouster_decoder src/decoder.cpp)
target_link_libraries(ouster_decoder PRIVATE ouster_decoder_base)

add_executable(ouster_driver src/driver.cpp)
target_link_libraries(ouster_driver PRIVATE ouster_decoder_base)

find_package(benchmark)
if(benchmark_FOUND)
  find_package(TBB REQUIRED)
  find_package(Eigen3 REQUIRED)
  add_executable(pcl_bench src/bench.cpp)
  target_link_libraries(
    pcl_bench PRIVATE ouster_decoder_base TBB::tbb benchmark::benchmark
                      benchmark::benchmark_main Eigen3::Eigen)
endif()
