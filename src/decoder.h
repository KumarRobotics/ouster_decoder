#pragma once

#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/core/mat.hpp>

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

namespace ouster_decoder {

class Decoder {
 public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  explicit Decoder(const ros::NodeHandle& pnh);

  // Non-copyable
  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  /// Callbacks
  void LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg);
  void ImuPacketCb(const ouster_ros::PacketMsg& imu_msg);

 private:
  void Timing(const ros::Time& start) const;

  /// Decode lidar packet
  void DecodeLidar(const uint8_t* const packet_puf);

  /// Whether we have had enough data for a full scan (to publish)
  bool HaveFullScan() const noexcept { return curr_col_ >= image_.cols; }

  /// Allocate storage that will be reused
  void Allocate(int rows, int cols);

  /// Publish messages
  void Publish();

  // ros
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber lidar_sub_, imu_sub_;
  ros::Publisher cloud_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  image_transport::Publisher dbg_range_pub_, dbg_intensity_pub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::string lidar_frame_, imu_frame_;

  // ouster
  ouster_ros::sensor::sensor_info info_;
  ouster_ros::sensor::packet_format const* pf_;

  // data
  cv::Mat image_;
  CloudT cloud_;
  int curr_col_{0};               // current column
  double dt_col_{};               // time between two columns
  double dt_packet_{};            // time between two packets
  double d_azimuth_{};            // delta azimuth radian
  std::vector<uint64_t> ts_;      // all time stamps (nanosecond)
  std::vector<double> azimuths_;  // all azimuth angles (radian)
};

}  // namespace ouster_decoder
