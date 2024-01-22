/* 
* Header file for decoder.cpp
*/

#ifndef DECODER_H
#define DECODER_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"

#include "lidar.h"

constexpr double kDefaultGravity = 9.807;

class Decoder 
{
 public:
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
  /// Initialize all parameters
  void InitParams();
  /// Initialize ouster related stuff
  void InitOuster();
  void InitModel(const std::string& metadata);
  void InitScan(const LidarModel& model);
  void SendTransform(const LidarModel& model);

  /// Whether we are still waiting for alignment to mid 0
  [[nodiscard]] bool NeedAlign(int mid);

  /// Publish messages
  void PublishAndReset();

  /// Record processing time of lidar callback, print warning if it exceeds time
  /// between two packets
  void Timing(const ros::Time& start) const;

  // ros
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber lidar_sub_, imu_sub_, meta_sub_;
  ros::Publisher cloud_pub_, imu_pub_;
  ros::Publisher range_pub_, signal_pub_;
  image_transport::CameraPublisher camera_pub_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
  std::string sensor_frame_, lidar_frame_, imu_frame_;

  // data
  LidarScan scan_;
  LidarModel model_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;

  // params
  double gravity_{};              // gravity
  bool replay_{false};            // replay mode will reinitialize on jump
  bool need_align_{true};         // whether to align scan
  double acc_noise_var_{};        // discrete time acc noise variance
  double gyr_noise_var_{};        // discrete time gyr noise variance
  double vis_signal_scale_{1.0};  // scale signal visualization
}; 
#endif
