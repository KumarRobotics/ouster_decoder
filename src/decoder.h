#pragma once

#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/core/mat.hpp>

#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

namespace ouster_decoder {

struct LidarData {
  float range{};            // range in meter
  float theta{};            // azimuth angle
  uint16_t shift{};         // pixel shift
  uint16_t signal{};        // signal intensity photon
  uint16_t ambient{};       // NIR photons
  uint16_t reflectivity{};  // calibrated reflectivity
} __attribute__((packed));

static_assert(sizeof(LidarData) == sizeof(float) * 4,
              "Size of Data must be 4 floats (16 bytes)");

struct LidarModel {
  LidarModel() = default;
  explicit LidarModel(const ouster_ros::sensor::sensor_info& info);

  int rows{};                     // number of beams
  int cols{};                     // cols of a full scan
  int freq{};                     // frequency
  double dt_meas{};               // delta time between two columns
  double d_azimuth{};             // delta angle between two columns
  double beam_offset{};           // distance between beam to origin
  std::vector<double> azimuths;   // azimuths offset angles
  std::vector<double> altitudes;  // altitude angles, high to low
  std::vector<int> pixel_shifts;  // offset pixel count
  std::string prod_line;          // produnction line

  void ToCameraInfo(sensor_msgs::CameraInfo& cinfo);
};

class Decoder {
 public:
  using PointT = pcl::PointXYZRGBL;
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
  /// Initialize ros related stuff (frame, publisher, subscriber)
  void InitRos();
  /// Send static transforms
  void SendTransform();

  /// Initialize ouster related stuff
  void InitOuster();
  /// Initialize all parameters
  void InitParams();
  /// Allocate storage that will be reused
  void Allocate(int rows, int cols);

  /// Decode lidar packet
  void DecodeLidar(const uint8_t* const packet_buf);
  void DecodeColumn(const uint8_t* const col_buf);
  /// Decode imu packet
  auto DecodeImu(const uint8_t* const packet_buf) -> sensor_msgs::Imu;

  /// Whether we have had enough data to publish
  bool ShouldPublish() const noexcept { return curr_col_ >= image_.cols; }
  /// Publish messages
  void Publish();
  /// Reset cached data
  void Reset();

  /// Record processing time of lidar callback, print warning if it exceeds time
  /// between two packets
  void Timing(const ros::Time& start) const;

  // ros
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber lidar_sub_, imu_sub_;
  ros::Publisher cloud_pub_, imu_pub_;
  ros::Publisher range_pub_;
  image_transport::CameraPublisher camera_pub_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
  std::string sensor_frame_, lidar_frame_, imu_frame_;

  // ouster
  ouster_ros::sensor::sensor_info info_;
  ouster_ros::sensor::packet_format const* pf_;

  // data
  cv::Mat image_;
  CloudT cloud_;
  std::vector<uint64_t> timestamps_;  // all time stamps (nanosecond)

  // params
  bool align_{};
  bool destagger_{};  // destagger image
  int curr_col_{0};   // current column
  int curr_scan_{0};  // current subscan
  double gravity_{};  // gravity
  double dt_packet_{};
  double min_range_{};

  LidarModel model_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;
};

}  // namespace ouster_decoder
