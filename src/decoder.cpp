#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lidar.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"

namespace ouster_decoder {

namespace os = ouster_ros::sensor;
namespace sm = sensor_msgs;

constexpr double kDefaultGravity = 9.807;  // [m/s^2] earth gravity

/// @brief Decoder node
class Decoder {
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
  [[nodiscard]] bool CheckAlign(int mid);

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
  sm::CameraInfoPtr cinfo_msg_;

  // params
  double gravity_{};        // gravity
  bool replay_{false};      // replay mode will reinitialize on jump
  bool need_align_{true};   // whether to align scan
  double acc_noise_var_{};  // discrete time acc noise variance
  double gyr_noise_var_{};  // discrete time gyr noise variance
};

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  InitParams();
  InitRos();
  InitOuster();
}

void Decoder::InitRos() {
  // Subscribers, queue size is 1 second
  lidar_sub_ =
      pnh_.subscribe("lidar_packets", 640, &Decoder::LidarPacketCb, this);
  imu_sub_ = pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  ROS_INFO_STREAM("Subscribing lidar packets: " << lidar_sub_.getTopic());
  ROS_INFO_STREAM("Subscribing imu packets: " << imu_sub_.getTopic());

  // Publishers
  camera_pub_ = it_.advertiseCamera("image", 10);
  cloud_pub_ = pnh_.advertise<sm::PointCloud2>("cloud", 10);
  imu_pub_ = pnh_.advertise<sm::Imu>("imu", 100);
  range_pub_ = pnh_.advertise<sm::Image>("range", 5);
  signal_pub_ = pnh_.advertise<sm::Image>("signal", 5);

  // Frames
  sensor_frame_ = pnh_.param<std::string>("sensor_frame", "os_sensor");
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os_lidar");
  imu_frame_ = pnh_.param<std::string>("imu_frame", "os_imu");
  ROS_INFO_STREAM("Sensor frame: " << sensor_frame_);
  ROS_INFO_STREAM("Lidar frame: " << lidar_frame_);
  ROS_INFO_STREAM("Imu frame: " << imu_frame_);
}

void Decoder::InitParams() {
  replay_ = pnh_.param<bool>("replay", false);
  ROS_INFO("Replay: %s", replay_ ? "true" : "false");
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  ROS_INFO("Gravity: %f", gravity_);
  scan_.destagger = pnh_.param<bool>("destagger", false);
  ROS_INFO("Destagger: %s", scan_.destagger ? "true" : "false");
  scan_.min_range = pnh_.param<double>("min_range", 0.5);
  scan_.max_range = pnh_.param<double>("max_range", 127.0);
  scan_.range_scale = pnh_.param<double>("range_scale", 512.0);
  ROS_INFO("Range: [%f, %f], scale: %f",
           scan_.min_range,
           scan_.max_range,
           scan_.range_scale);
  if (scan_.max_range * scan_.range_scale >
      static_cast<double>(ImageData::kMaxUint16)) {
    throw std::domain_error("max range exceeds representation");
  }

  acc_noise_var_ = pnh_.param<double>("acc_noise_std", 0.0023);
  gyr_noise_var_ = pnh_.param<double>("gyr_noise_std", 0.00026);
  // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
  acc_noise_var_ = std::pow(acc_noise_var_, 2) * 100.0;
  gyr_noise_var_ = std::pow(gyr_noise_var_, 2) * 100.0;
  ROS_INFO("Discrete time acc noise var: %f, gyr nosie var: %f",
           acc_noise_var_,
           gyr_noise_var_);
}

void Decoder::InitOuster() {
  ROS_INFO_STREAM("=== Initializing Ouster Decoder ===");
  // wait for service
  auto client = pnh_.serviceClient<ouster_ros::OSConfigSrv>("os_config");

  // NOTE: it is possible that in replay mode, the service was shutdown and
  // re-advertised. If the client call is too soon, then we risk getting the old
  // metadata. It is also possible that the call would fail because the driver
  // side hasn't finished re-advertising. Therefore in replay mode, we add a
  // small delay before calling the service.
  if (replay_) {
    ros::Duration(0.1).sleep();
  }

  client.waitForExistence();

  ouster_ros::OSConfigSrv cfg{};
  // Initialize everything if service call is successful
  if (client.call(cfg)) {
    InitModel(cfg.response.metadata);
    InitScan(model_);
    SendTransform(model_);
  } else {
    ROS_ERROR_STREAM(client.getService() << " call failed, abort.");
    ros::shutdown();
  }
}

void Decoder::InitModel(const std::string& metadata) {
  // parse metadata into lidar model
  model_ = LidarModel{metadata};
  ROS_INFO("Lidar mode %s: %d x %d @ %d hz",
           os::to_string(model_.info.mode).c_str(),
           model_.rows,
           model_.cols,
           model_.freq);
  ROS_INFO("Columns per packet: %d, Pixels per column: %d",
           model_.pf->columns_per_packet,
           model_.pf->pixels_per_column);

  // Generate partial camera info message
  cinfo_msg_ = boost::make_shared<sm::CameraInfo>();
  model_.UpdateCameraInfo(*cinfo_msg_);
}

void Decoder::InitScan(const LidarModel& model) {
  int num_subscans = pnh_.param<int>("divide", 1);
  // Make sure cols is divisible by num_subscans
  if (num_subscans < 1 || model.cols % num_subscans != 0) {
    throw std::domain_error(
        "num subscans is not divisible by cols: " + std::to_string(model.cols) +
        " / " + std::to_string(num_subscans));
  }

  // Each block has 16 cols, make sure we dont divide into anything smaller
  num_subscans = std::min(num_subscans, model.cols / 16);

  const int subscan_cols = model.cols / num_subscans;
  ROS_INFO("Subscan %d x %d, total %d", model.rows, subscan_cols, num_subscans);
  scan_.Allocate(model.rows, subscan_cols);
}

void Decoder::SendTransform(const LidarModel& model) {
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      model.info.imu_to_sensor_transform, sensor_frame_, imu_frame_));
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      model.info.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));
}

void Decoder::PublishAndReset() {
  std_msgs::Header header;
  header.frame_id = lidar_frame_;
  header.stamp.fromNSec(scan_.times.back());  // use time of the last column

  // Publish image and camera_info
  // cinfo stores information about the full sweep, while roi stores information
  // about the subscan
  const auto image_msg =
      cv_bridge::CvImage(header, "32FC4", scan_.image).toImageMsg();
  cinfo_msg_->header = header;
  // Update camera info roi with scan
  scan_.UpdateCinfo(*cinfo_msg_);
  camera_pub_.publish(image_msg, cinfo_msg_);

  // Publish range image on demand
  if (range_pub_.getNumSubscribers() > 0 ||
      signal_pub_.getNumSubscribers() > 0) {
    // cast image as 8 channel short so that we can extract the last 2 as range
    // and signal
    cv::Mat image16u(
        scan_.image.rows, scan_.image.cols, CV_16UC(8), scan_.image.data);

    if (range_pub_.getNumSubscribers() > 0) {
      cv::Mat range;
      cv::extractChannel(image16u, range, 6);
      range_pub_.publish(
          cv_bridge::CvImage(header, "16UC1", range).toImageMsg());
    }

    if (signal_pub_.getNumSubscribers() > 0) {
      cv::Mat signal;
      cv::extractChannel(image16u, signal, 7);
      // multiply by 32 for visualization purposes
      signal_pub_.publish(
          cv_bridge::CvImage(header, "16UC1", signal * 32).toImageMsg());
    }
  }

  // Publish cloud
  if (cloud_pub_.getNumSubscribers() > 0) {
    scan_.cloud.header = header;
    cloud_pub_.publish(scan_.cloud);
  }

  // Increment subscan counter
  ++scan_.iscan;
  // Reset cached data after publish
  scan_.SoftReset(model_.cols);
}

void Decoder::Timing(const ros::Time& t_start) const {
  const auto t_end = ros::Time::now();
  const auto t_proc = (t_end - t_start).toSec();
  const auto ratio = t_proc / model_.dt_packet;
  if (ratio > 5) {
    ROS_WARN("Proc time: %.4f ms, meas time: %.4f ms, ratio: %.1f%%",
             t_proc * 1e3,
             model_.dt_packet * 1e3,
             ratio * 100);
  }
  ROS_DEBUG_THROTTLE(1,
                     "Proc time: %.4f ms, meas time: %.4f ms, ratio: %.1f%%",
                     t_proc * 1e3,
                     model_.dt_packet * 1e3,
                     ratio * 100);
}

bool Decoder::CheckAlign(int mid) {
  if (need_align_ && mid == 0) {
    need_align_ = false;
    ROS_INFO("Align start of scan to mid %d, icol in scan %d", mid, scan_.icol);
  }
  return need_align_;
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto t0 = ros::Time::now();
  const auto* packet_buf = lidar_msg.buf.data();
  const auto& pf = *model_.pf;
  //    const int fid = pf.frame_id(packet_buf);

  for (int col = 0; col < pf.columns_per_packet; ++col) {
    // Get column buffer
    const uint8_t* const col_buf = pf.nth_col(col, packet_buf);
    const int fid = pf.col_frame_id(col_buf);
    const int mid = pf.col_measurement_id(col_buf);

    // If we set need_align to true then this will wait for mid = 0 to
    // start a scan
    if (CheckAlign(mid)) {
      continue;
    }

    // The invariant here is that scan_.icol will always point at the current
    // column to be filled at the beginning of the loop

    // Sometimes the lidar packet will jump forward by a large chunk, we handle
    // this case here
    const auto uid = model_.Uid(fid, mid);
    const auto jump = scan_.DetectJump(uid);
    if (jump == 0) {
      // Data arrived as expected, decode and forward
      scan_.DecodeColumn(col_buf, model_);
      if (scan_.IsFull()) {
        PublishAndReset();
        Timing(t0);
      }
    } else if (0 < jump && jump <= model_.cols) {
      ROS_WARN("Packet jumped to f%d:m%d by %d columns.", fid, mid, jump);
      // Detect a jump, we need to forward scan icol by the same amount as jump
      // We could directly increment icol and publish if necessary, but
      // this will require us to zero the whole cloud at publish time which is
      // very time-consuming. Therefore, we choose to advance icol one by one
      // and zero out each column in the point cloud
      for (int i = 0; i < jump; ++i) {
        // zero cloud column at current col and then increment
        scan_.InvalidateColumn(model_.dt_col);
        // It is possible that this jump will span two scans, so if that is
        // the case, we need to publish the previous scan before moving forward
        if (scan_.IsFull()) {
          ROS_WARN("Jumped into a new scan, need to publish the previous one");
          PublishAndReset();
          Timing(t0);
        }
      }
    } else {
      // Handle backward jump or large forward jump
      ROS_ERROR("Packet jumped to f%d:m%d by %d columns.", fid, mid, jump);
      if (replay_) {
        ROS_WARN("In replay mode, re-initialize...");
        need_align_ = true;
        scan_.HardReset();
        // Also need to reinitialize everything since it is possible that it is
        // a different dataset
        InitOuster();
      } else {
        ROS_FATAL("Not in replay mode, shutting down...");
        ros::shutdown();
      }
      return;
    }
  }
}

void Decoder::ImuPacketCb(const ouster_ros::PacketMsg& imu_msg) {
  const auto* buf = imu_msg.buf.data();
  const auto& pf = *model_.pf;

  sm::Imu m;
  m.header.stamp.fromNSec(pf.imu_gyro_ts(buf));
  m.header.frame_id = imu_frame_;

  // Invalidate orientation data since we don't have it
  // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
  auto& q = m.orientation;
  q.x = q.y = q.z = q.w = 0;
  m.orientation_covariance[0] = -1;

  auto& a = m.linear_acceleration;
  a.x = pf.imu_la_x(buf) * gravity_;
  a.y = pf.imu_la_y(buf) * gravity_;
  a.z = pf.imu_la_z(buf) * gravity_;

  auto& w = m.angular_velocity;
  w.x = Deg2Rad(pf.imu_av_x(buf));
  w.y = Deg2Rad(pf.imu_av_y(buf));
  w.z = Deg2Rad(pf.imu_av_z(buf));

  for (int i = 0; i < 9; i += 4) {
    m.linear_acceleration_covariance[i] = acc_noise_var_;
    m.angular_velocity_covariance[i] = gyr_noise_var_;
  }

  imu_pub_.publish(m);
}

}  // namespace ouster_decoder

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_decoder");

  ouster_decoder::Decoder node(ros::NodeHandle("~"));
  ros::spin();

  return 0;
}
