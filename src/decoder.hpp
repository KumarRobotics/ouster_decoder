#pragma once

// ros
#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf2_ros/static_transform_broadcaster.h>

// message/service
#include "ouster_ros/PacketMsg.h"

namespace ouster_decoder {

class Decoder {
 public:
  explicit Decoder(const ros::NodeHandle& pnh);

  // Non-copyable
  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  // Callbacks
  void LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg);
  void ImuPacketCb(const ouster_ros::PacketMsg& imu_msg);

 private:
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Publisher cloud_pub_, imu_pub_;
  image_transport::CameraPublisher camera_pub_;
  ros::Subscriber lidar_packet_sub_, imu_packet_sub_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

}  // namespace ouster_decoder
