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

  // No copy no move
  Decoder(const Decoder&) = delete;
  Decoder& operator=(const Decoder&) = delete;
  Decoder(Decoder&&) = delete;
  Decoder& operator=(Decoder&&) = delete;

  /// Callbacks
  void LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg);
  void ImuPacketCb(const ouster_ros::PacketMsg& imu_msg);

 private:
  /// Decode lidar packet
  void DecodeLidar(const uint8_t* const packet_buf);

  /// Decode imu packet
  auto DecodeImu(const uint8_t* const packet_buf) -> sensor_msgs::Imu;

  /// Whether we have had enough data for a full scan (to publish)
  bool HaveFullScan() const noexcept { return curr_col_ >= image_.cols; }

  /// Allocate storage that will be reused
  void Allocate(int rows, int cols);

  /// Publish messages
  void Publish();

  /// Send static transforms
  void SendTransform();

  /// Record processing time of lidar callback, print warning if it exceeds time
  /// between two packets
  void Timing(const ros::Time& start) const;

  // ros
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber lidar_sub_, imu_sub_;
  ros::Publisher cloud_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
  std::string sensor_frame_, lidar_frame_, imu_frame_;

  // ouster
  ouster_ros::sensor::sensor_info info_;
  ouster_ros::sensor::packet_format const* pf_;

  // data
  cv::Mat image_;
  CloudT cloud_;
  int curr_col_{0};                   // current column
  double dt_col_{};                   // time between two columns
  double dt_packet_{};                // time between two packets
  double d_azimuth_{};                // delta azimuth radian
  double gravity_{};                  // gravity
  std::vector<uint64_t> timestamps_;  // all time stamps (nanosecond)
  std::vector<double> azimuths_;      // all azimuth angles (radian)
};

}  // namespace ouster_decoder
