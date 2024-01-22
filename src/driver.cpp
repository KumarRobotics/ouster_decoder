/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Example node to publish raw sensor output on ROS topics
 *
 * ROS Parameters
 * sensor_hostname: hostname or IP in dotted decimal form of the sensor
 * udp_dest: hostname or IP where the sensor will send data packets
 * lidar_port: port to which the sensor should send lidar data
 * imu_port: port to which the sensor should send imu data
 */

// This is a modified version of ouster_ros/os_node.cpp
// It is intended to have the same behavior as os_node.
// The only difference is that we also advertise the metadata message.
// The logic here was kept the same in the 2024 refactor.


#include <ros/ros.h>
#include <std_msgs/String.h>

#include <fstream>
#include <sstream>
#include <string>

#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os_ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using OsGetMetadata = ouster_ros::GetMetadata;
namespace sensor = ouster::sensor;

// fill in values that could not be parsed from metadata
void populate_metadata_defaults(sensor::sensor_info& info,
                                sensor::lidar_mode specified_lidar_mode) {
  if (!info.name.size()) info.name = "UNKNOWN";

  if (!info.sn.size()) info.sn = "UNKNOWN";

  ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
  if (v == ouster::util::invalid_version)
    ROS_WARN("Unknown sensor firmware version; output may not be reliable");
  else if (v < sensor::min_version)
    ROS_WARN("Firmware < %s not supported; output may not be reliable",
             to_string(sensor::min_version).c_str());

  if (!info.mode) {
    ROS_WARN("Lidar mode not found in metadata; output may not be reliable");
    info.mode = specified_lidar_mode;
  }

  if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

  if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
    ROS_WARN("Beam angles not found in metadata; using design values");
    info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
    info.beam_altitude_angles = sensor::gen1_altitude_angles;
  }
}

// try to write metadata file
bool write_metadata(const std::string& meta_file, const std::string& metadata) {
  std::ofstream ofs;
  ofs.open(meta_file);
  ofs << metadata << std::endl;
  ofs.close();
  if (ofs) {
    ROS_INFO("Wrote metadata to %s", meta_file.c_str());
  } else {
    ROS_WARN(
        "Failed to write metadata to %s; check that the path is valid. If "
        "you provided a relative path, please note that the working "
        "directory of all ROS nodes is set by default to $ROS_HOME",
        meta_file.c_str());
    return false;
  }
  return true;
}

int connection_loop(ros::NodeHandle& nh,
                    sensor::client& cli,
                    const sensor::sensor_info& info) {
  auto lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
  auto imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);

  auto pf = sensor::get_format(info);

  PacketMsg lidar_packet, imu_packet;
  lidar_packet.buf.resize(pf.lidar_packet_size + 1);
  imu_packet.buf.resize(pf.imu_packet_size + 1);

  while (ros::ok()) {
    auto state = sensor::poll_client(cli);
    if (state == sensor::EXIT) {
      ROS_INFO("poll_client: caught signal, exiting");
      return EXIT_SUCCESS;
    }
    if (state & sensor::CLIENT_ERROR) {
      ROS_ERROR("poll_client: returned error");
      return EXIT_FAILURE;
    }
    if (state & sensor::LIDAR_DATA) {
      if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf))
        lidar_packet_pub.publish(lidar_packet);
    }
    if (state & sensor::IMU_DATA) {
      if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf))
        imu_packet_pub.publish(imu_packet);
    }
    ros::spinOnce();
  }
  return EXIT_SUCCESS;
}

void advertise_service(ros::NodeHandle& nh,
                       ros::ServiceServer& srv,
                       const sensor::sensor_info& info) {
  auto metadata = sensor::to_string(info);
  ROS_INFO("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
  ROS_INFO("%s sn: %s firmware rev: %s",
           info.prod_line.c_str(),
           info.sn.c_str(),
           info.fw_rev.c_str());
  if (srv) {
    // shutdown first and readvertise
    ROS_INFO("Shutting down %s service and re-advertise",
             srv.getService().c_str());
    srv.shutdown();
  }
  srv = nh.advertiseService<OsGetMetadata::Request, OsGetMetadata::Response>(
      "get_metadata",
      [metadata](OsGetMetadata::Request&, OsGetMetadata::Response& res) {
        if (metadata.empty()) return false;
        res.metadata = metadata;
        return true;
      });
  ROS_INFO("Advertise service to %s", srv.getService().c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_node");
  ros::NodeHandle nh("~");
  // Extra stuff
  ros::Publisher pub_meta;
  ros::Subscriber meta_sub;
  ros::ServiceServer srv;

  // empty indicates "not set" since roslaunch xml can't optionally set params
  auto hostname = nh.param("sensor_hostname", std::string{});
  auto udp_dest = nh.param("udp_dest", std::string{});
  auto lidar_port = nh.param("lidar_port", 0);
  auto imu_port = nh.param("imu_port", 0);
  auto replay = nh.param("replay", false);
  auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
  auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});

  std::string udp_profile_lidar_arg;
  nh.param<std::string>("udp_profile_lidar", udp_profile_lidar_arg, "");

  //   optional<sensor::UDPProfileLidar> udp_profile_lidar;
  //   if (udp_profile_lidar_arg.size()) {
  //     if (replay)
  //       ROS_WARN("UDP Profile Lidar set in replay mode. Will be ignored.");

  //     // set lidar profile from param
  //     udp_profile_lidar =
  //         sensor::udp_profile_lidar_of_string(udp_profile_lidar_arg);
  //     if (!udp_profile_lidar) {
  //       ROS_ERROR("Invalid udp profile lidar: %s",
  //       udp_profile_lidar_arg.c_str()); return EXIT_FAILURE;
  //     }
  //   }

  // set lidar mode from param
  sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
  if (lidar_mode_arg.size()) {
    if (replay) ROS_WARN("Lidar mode set in replay mode. Will be ignored");

    lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
    if (!lidar_mode) {
      ROS_ERROR("Invalid lidar mode %s", lidar_mode_arg.c_str());
      return EXIT_FAILURE;
    }
  }

  // set timestamp mode from param
  sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
  if (timestamp_mode_arg.size()) {
    if (replay) ROS_WARN("Timestamp mode set in replay mode. Will be ignored");

    timestamp_mode = sensor::timestamp_mode_of_string(timestamp_mode_arg);
    if (!timestamp_mode) {
      ROS_ERROR("Invalid timestamp mode %s", timestamp_mode_arg.c_str());
      return EXIT_FAILURE;
    }
  }

  // fall back to metadata file name based on hostname, if available
  auto meta_file = nh.param("metadata", std::string{});
  // if (!meta_file.size() && hostname.size()) meta_file = hostname + ".json";

  if (!replay && (!hostname.size() || !udp_dest.size())) {
    ROS_ERROR("Must specify both hostname and udp destination");
    return EXIT_FAILURE;
  }

  // ROS_INFO("Client version: %s", ouster::CLIENT_VERSION_FULL);

  if (replay) {
    ROS_INFO("Running in replay mode");

    auto meta_cb = [&nh, &srv](const std_msgs::String& str_msg) {
      auto info = sensor::parse_metadata(str_msg.data);
      advertise_service(nh, srv, info);
    };

    // populate info for config service
    try {
      if (meta_file.empty()) {
        meta_sub = nh.subscribe<std_msgs::String, const std_msgs::String&>(
            "metadata", 1, meta_cb);
        ROS_INFO("metadata is empty, subscribing to %s",
                 meta_sub.getTopic().c_str());

      } else {
        ROS_INFO("metadata file is given, using %s", meta_file.c_str());
        auto info = sensor::metadata_from_json(meta_file);
        advertise_service(nh, srv, info);
      }

      // just serve config service
      ros::spin();
      return EXIT_SUCCESS;
    } catch (const std::runtime_error& e) {
      ROS_ERROR("Error when running in replay mode: %s", e.what());
    }
  } else {
    ROS_INFO("Waiting for sensor %s to initialize ...", hostname.c_str());
    ROS_INFO("Sending data to %s", udp_dest.c_str());

    // use no-config version of init_client to allow for random ports
    // auto cli = sensor::init_client(hostname, lidar_port, imu_port);
    auto cli = sensor::init_client(
        hostname, udp_dest, lidar_mode, timestamp_mode, lidar_port, imu_port);

    if (!cli) {
      ROS_ERROR("Failed to initialize sensor at: %s", hostname.c_str());
      return EXIT_FAILURE;
    }
    ROS_INFO("Sensor initialized successfully");

    // write metadata file to cwd (usually ~/.ros)
    auto metadata = sensor::get_metadata(*cli);
    if (meta_file.empty()) {
      meta_file = hostname + ".json";  // hostname must be nonempty
      ROS_INFO("meta_file not given, use: %s", meta_file.c_str());
    }

    // write metadata file. If metadata_path is relative, will use cwd
    // (usually ~/.ros)
    if (!write_metadata(meta_file, metadata)) {
      ROS_ERROR("Exiting because of failure to write metadata path to %s",
                meta_file.c_str());
      return EXIT_FAILURE;
    }

    // populate sensor info
    auto info = sensor::parse_metadata(metadata);
    populate_metadata_defaults(info, sensor::MODE_UNSPEC);
    metadata = to_string(info);  // regenerate metadata

    // publish metadata
    pub_meta = nh.advertise<std_msgs::String>("metadata", 1, true);
    std_msgs::String meta_msg;
    meta_msg.data = metadata;
    pub_meta.publish(meta_msg);
    ROS_INFO("Publish metadata to %s", pub_meta.getTopic().c_str());

    srv = nh.advertiseService<OsGetMetadata::Request, OsGetMetadata::Response>(
        "get_metadata",
        [metadata](OsGetMetadata::Request&, OsGetMetadata::Response& res) {
          if (metadata.empty()) return false;
          res.metadata = metadata;
          return true;
        });

    ROS_INFO("Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
    ROS_INFO("%s sn: %s firmware rev: %s",
             info.prod_line.c_str(),
             info.sn.c_str(),
             info.fw_rev.c_str());

    // publish packet messages from the sensor
    return connection_loop(nh, *cli, info);
  }
}
