#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "lidar.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"

namespace ouster_decoder {

namespace os = ouster_ros::sensor;

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
  void MetadataCb(const std_msgs::String& meta_msg);

 private:
  /// Initialize ros related stuff (frame, publisher, subscriber)
  void InitRos();
  /// Initialize ouster related stuff
  void InitModel(const std::string& metadata);
  /// Initialize all parameters
  void InitParams();
  /// Send static transforms
  void SendTransform();

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
  ros::Subscriber lidar_sub_, imu_sub_;
  ros::Subscriber meta_sub_;
  ros::Publisher cloud_pub_, imu_pub_;
  ros::Publisher range_pub_;
  image_transport::CameraPublisher camera_pub_;
  tf2_ros::StaticTransformBroadcaster static_tf_;
  std::string sensor_frame_, lidar_frame_, imu_frame_;

  // data
  LidarScan scan_;
  LidarModel model_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;

  // params
  double gravity_{};        // gravity
  bool strict_{false};      // strict mode will die if data jumps backwards
  bool need_align_{true};   // whether to align scan
  double acc_noise_var_{};  // discrete time acc noise variance
  double gyr_noise_var_{};  // discrete time gyr noise variance
};

Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  auto client = pnh_.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  // wait for service
  if (client.waitForExistence()) {
    ROS_INFO_STREAM(client.getService() << " is available");
    ouster_ros::OSConfigSrv cfg{};
    // Initialize everything if service call is successful
    if (client.call(cfg)) {
      InitModel(cfg.response.metadata);
      InitRos();
      InitParams();
      SendTransform();
    } else {
      ROS_ERROR_STREAM(client.getService() << " call failed, shutdown");
      ros::shutdown();
    }
  }
}

void Decoder::InitModel(const std::string& metadata) {
  // parse metadata into lidar model
  model_ = LidarModel{metadata};
  ROS_INFO_STREAM("Lidar mode: " << os::to_string(model_.info.mode));
  ROS_INFO("Lidar: %d x %d @ %d hz", model_.rows, model_.cols, model_.freq);
  ROS_INFO("Columns per packet: %d", model_.pf->columns_per_packet);
  ROS_INFO("Pixels per column: %d", model_.pf->pixels_per_column);

  // Generate partial camera info message
  cinfo_msg_ = boost::make_shared<sensor_msgs::CameraInfo>();
  model_.UpdateCameraInfo(*cinfo_msg_);
}

void Decoder::InitParams() {
  strict_ = pnh_.param<bool>("strict", false);
  ROS_INFO("Strict: %s", strict_ ? "true" : "false");
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  ROS_INFO("Gravity: %f", gravity_);
  scan_.destagger = pnh_.param<bool>("destagger", false);
  ROS_INFO("Destagger: %s", scan_.destagger ? "true" : "false");
  scan_.min_range = pnh_.param<double>("min_range", 0.5);
  scan_.max_range = pnh_.param<double>("max_range", 128.0);
  ROS_INFO("Range: [%f, %f]", scan_.min_range, scan_.max_range);

  int num_subscans = pnh_.param<int>("divide", 1);
  // Make sure cols is divisible by num_subscans
  if (num_subscans < 1 || model_.cols % num_subscans != 0) {
    throw std::runtime_error("num subscans is not divisible by cols: " +
                             std::to_string(model_.cols) + " / " +
                             std::to_string(num_subscans));
  }

  const int scan_cols = model_.cols / num_subscans;
  ROS_INFO("Subscan %d x %d, total %d", model_.rows, scan_cols, num_subscans);
  scan_.Allocate(model_.rows, scan_cols);

  acc_noise_var_ = pnh_.param<double>("acc_noise_std", 0.0023);
  gyr_noise_var_ = pnh_.param<double>("gyr_noise_std", 0.00026);
  // https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
  acc_noise_var_ = std::pow(acc_noise_var_, 2) * 100.0;
  gyr_noise_var_ = std::pow(gyr_noise_var_, 2) * 100.0;
  ROS_INFO("Discrete time acc noise var: %f, gyr nosie var: %f",
           acc_noise_var_,
           gyr_noise_var_);
}

void Decoder::InitRos() {
  // Subscribers, queue size is 1 second
  lidar_sub_ =
      pnh_.subscribe("lidar_packets", 640, &Decoder::LidarPacketCb, this);
  imu_sub_ = pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  ROS_INFO_STREAM("Subscribing lidar packets from: " << lidar_sub_.getTopic());
  ROS_INFO_STREAM("Subscribing imu packets from: " << imu_sub_.getTopic());

  // Publishers
  imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("imu", 100);
  cloud_pub_ = pnh_.advertise<CloudT>("cloud", 10);
  camera_pub_ = it_.advertiseCamera("image", 10);
  range_pub_ = pnh_.advertise<sensor_msgs::Image>("range", 1);

  // Frames
  sensor_frame_ = pnh_.param<std::string>("sensor_frame", "os_sensor");
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os_lidar");
  imu_frame_ = pnh_.param<std::string>("imu_frame", "os_imu");
  ROS_INFO_STREAM("Sensor frame: " << sensor_frame_);
  ROS_INFO_STREAM("Lidar frame: " << lidar_frame_);
  ROS_INFO_STREAM("Imu frame: " << imu_frame_);
}

void Decoder::SendTransform() {
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      model_.info.imu_to_sensor_transform, sensor_frame_, imu_frame_));
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      model_.info.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));
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
  cinfo_msg_->binning_x = scan_.iscan;
  cinfo_msg_->binning_y = model_.cols / scan_.cols();
  scan_.UpdateRoi(cinfo_msg_->roi);
  camera_pub_.publish(image_msg, cinfo_msg_);

  // Publish range image on demand
  if (range_pub_.getNumSubscribers() > 0) {
    cv::Mat range;
    cv::extractChannel(scan_.image, range, 3);
    range_pub_.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());
  }

  // Publish cloud
  pcl_conversions::toPCL(header, scan_.cloud.header);
  cloud_pub_.publish(scan_.cloud);

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
    ROS_WARN("Proc time: %f ms, meas time: %f ms, ratio: %f",
             t_proc * 1e3,
             model_.dt_packet * 1e3,
             ratio);
  }
  ROS_DEBUG_THROTTLE(1,
                     "Proc time: %f ms, meas time: %f ms, ratio: %f",
                     t_proc * 1e3,
                     model_.dt_packet * 1e3,
                     ratio);
}

bool Decoder::CheckAlign(int mid) {
  if (need_align_ && mid == 0) {
    need_align_ = false;
    ROS_INFO("Align start of scan to mid %d, icol in scan %d", mid, scan_.icol);
  }
  return need_align_;
}

// This is currently disabled
void Decoder::MetadataCb(const std_msgs::String& meta_msg) {
  InitModel(meta_msg.data);
  InitRos();
  InitParams();
  SendTransform();
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto t0 = ros::Time::now();
  const auto* packet_buf = lidar_msg.buf.data();
  const auto& pf = *model_.pf;

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
    const auto jump = scan_.DetectJump(model_.Uid(fid, mid));
    if (jump == 0) {
      // Data arrived as expected, decode and forward
      scan_.DecodeColumn(col_buf, model_);
      if (scan_.IsFull()) {
        PublishAndReset();
        Timing(t0);
      }
    } else if (0 < jump && jump < model_.cols) {
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
      ROS_ERROR("Packet jumped to f%d:m%d by %d columns.", fid, mid, jump);
      if (strict_) {
        ROS_FATAL("In strict mode, shutting down...");
        ros::shutdown();
      } else {
        ROS_WARN("Not in strict mode, reset internal state and wait for align");
        need_align_ = true;
        scan_.HardReset();
      }
      return;
    }
  }
}

void Decoder::ImuPacketCb(const ouster_ros::PacketMsg& imu_msg) {
  const auto* buf = imu_msg.buf.data();
  const auto& pf = *model_.pf;

  sensor_msgs::Imu m;
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
