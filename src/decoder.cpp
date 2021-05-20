#include "decoder.h"

#include "ouster_ros/OSConfigSrv.h"

namespace ouster_decoder {

namespace os = ouster_ros::sensor;

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  // Call service to retrieve sensor info, this must be done first
  ouster_ros::OSConfigSrv cfg{};
  auto client = pnh_.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  client.waitForExistence();
  if (!client.call(cfg)) {
    throw std::runtime_error("Calling config service failed");
  }

  ROS_INFO("Parsing OSConfig");
  info_ = os::parse_metadata(cfg.response.metadata);
  ROS_DEBUG_STREAM("Metadata: " << os::to_string(info_));
  ROS_INFO_STREAM("Lidar mode: " << os::to_string(info_.mode));

  lidar_sub_ =
      pnh_.subscribe("lidar_packets", 2048, &Decoder::LidarPacketCb, this);
  imu_sub_ = pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  ROS_INFO("Subscribing lidar from: %s", lidar_sub_.getTopic().c_str());
  ROS_INFO("Subscribing imu from: %s", imu_sub_.getTopic().c_str());
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  //  ROS_INFO("Lidar packet");
}

void Decoder::ImuPacketCb(const ouster_ros::PacketMsg& imu_msg) {
  //  ROS_INFO("Imu packet");
}

}  // namespace ouster_decoder

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_decoder");

  ouster_decoder::Decoder node(ros::NodeHandle("~"));
  ros::spin();

  return 0;
}
