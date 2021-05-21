#include "decoder.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "ouster_ros/OSConfigSrv.h"

namespace ouster_decoder {

namespace {

constexpr double kPi = M_PI;
constexpr double kTau = 2 * kPi;
constexpr double kMmToM = 0.001f;
constexpr double kDefaultGravity = 9.807;  // [m/s^2] earth gravity
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / kPi; }

// Convert a vector of double from deg to rad
void TransformDeg2RadInPlace(std::vector<double>& vec) {
  std::transform(vec.begin(), vec.end(), vec.begin(), deg2rad);
}

}  // namespace

namespace os = ouster_ros::sensor;

// Decoder
// Calls os_config service to get sensor info.
// Subscribes to lidar and imu packets and publishes image, camera info, point
// cloud, imu and static transforms
//
// Image:
// Decoded lidar packets are stored in a cv::Mat and published as
// sensor_msgs::Image. The image is a 3-channel float image. The channels are
// [range, azimuth, intensity]. The image is stored in staggered form, meaning
// pixels in each column will have the same timestamp
//
// CameraInfo:
// Auxillary information is stored in sensor_msgs::CameraInfo.
// D stores beam_altitude_angles
// The time between two columns is in K[0].
// The angle between two columns is in R[0].
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

  // Subscribers
  lidar_sub_ =
      pnh_.subscribe("lidar_packets", 64, &Decoder::LidarPacketCb, this);
  imu_sub_ = pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  ROS_INFO_STREAM("Subscribing lidar from: " << lidar_sub_.getTopic());
  ROS_INFO_STREAM("Subscribing imu from: " << imu_sub_.getTopic());

  // Publishers
  cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("imu", 100);
  camera_pub_ = it_.advertiseCamera("image", 10);

  // Frames
  sensor_frame_ = pnh_.param<std::string>("sensor_frame", "os_sensor");
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os_lidar");
  imu_frame_ = pnh_.param<std::string>("imu_frame", "os_imu");
  ROS_INFO_STREAM("Sensor frame: " << sensor_frame_);
  ROS_INFO_STREAM("Lidar frame: " << lidar_frame_);
  ROS_INFO_STREAM("Imu frame: " << imu_frame_);

  // Params
  int cols = os::n_cols_of_lidar_mode(info_.mode);
  int rows = info_.beam_altitude_angles.size();
  int freq = os::frequency_of_lidar_mode(info_.mode);
  ROS_INFO("Lidar mode: %d x %d @ %d hz", rows, cols, freq);

  pf_ = &os::get_format(info_);
  ROS_INFO("columns per packet: %d", pf_->columns_per_packet);
  ROS_INFO("pixels per column: %d", pf_->pixels_per_column);

  // Data
  dt_col_ = 1.0 / freq / cols;                     // dt two col
  dt_packet_ = dt_col_ * pf_->columns_per_packet;  // dt two packet
  d_azimuth_ = kTau / cols;                        // angle two columns
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  ROS_INFO("Gravity: %f", gravity_);

  // Transform angle from degree to radian
  TransformDeg2RadInPlace(info_.beam_altitude_angles);
  TransformDeg2RadInPlace(info_.beam_azimuth_angles);

  Allocate(rows, cols);
  SendTransform();
}

void Decoder::SendTransform() {
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      info_.imu_to_sensor_transform, sensor_frame_, imu_frame_));

  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      info_.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));
}

void Decoder::Allocate(int rows, int cols) {
  image_.create(rows, cols, CV_32FC3);
  cloud_ = CloudT(cols, rows);  // point cloud ctor takes width and height
  timestamps_.resize(cols);
  azimuths_.resize(cols);
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto start = ros::Time::now();
  DecodeLidar(lidar_msg.buf.data());

  // We have data for the entire range image
  if (HaveFullScan()) {
    curr_col_ = 0;
    Publish();
  }

  Timing(start);
}

void Decoder::ImuPacketCb(const ouster_ros::PacketMsg& imu_msg) {
  imu_pub_.publish(DecodeImu(imu_msg.buf.data()));
}

void Decoder::Publish() {
  std_msgs::Header header;
  header.frame_id = lidar_frame_;
  header.stamp.fromNSec(timestamps_.front());

  // Publish image and camera_info
  auto image_msg = cv_bridge::CvImage(header, "32FC3", image_).toImageMsg();
  auto cinfo_msg = boost::make_shared<sensor_msgs::CameraInfo>();
  cinfo_msg->header = header;
  cinfo_msg->height = image_msg->height;
  cinfo_msg->width = image_msg->width;
  cinfo_msg->distortion_model = info_.prod_line;
  cinfo_msg->D = info_.beam_altitude_angles;
  cinfo_msg->K[0] = dt_col_;     // delta time between two columns
  cinfo_msg->R[0] = d_azimuth_;  // delta angle between two columns
  camera_pub_.publish(image_msg, cinfo_msg);

  // Publish cloud
  pcl_conversions::toPCL(header, cloud_.header);
  cloud_pub_.publish(cloud_);
}

void Decoder::Timing(const ros::Time& start) const {
  const auto t_end = ros::Time::now();
  const auto t_proc = (t_end - start).toSec();
  const auto ratio = t_proc / dt_packet_;
  if (ratio > 1.2) {
    ROS_WARN("Proc time: %f ms, meas time: %f ms, ratio: %f",
             t_proc * 1e3,
             dt_packet_ * 1e3,
             ratio);
  }
}

void Decoder::DecodeLidar(const uint8_t* const packet_buf) {
  const auto& pf = *pf_;

  for (int icol = 0; icol < pf.columns_per_packet; ++icol, ++curr_col_) {
    // Get all the relevant data
    const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
    const uint16_t meas_id = pf.col_measurement_id(col_buf);
    const uint16_t frame_id = pf.col_frame_id(col_buf);
    const uint64_t t_ns = pf.col_timestamp(col_buf);
    //    const uint32_t encoder = pf.col_encoder(col_buf);  // not necessary
    const uint32_t status = pf.col_status(col_buf);
    const bool valid = (status == 0xffffffff);

    // Compute azimuth angle theta0, this should always be valid
    const auto theta0 = kTau - meas_id * d_azimuth_;
    timestamps_.at(curr_col_) = t_ns;
    azimuths_.at(curr_col_) = theta0;

    for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
      const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

      const auto range = pf.px_range(px_buf) * kMmToM;
      const auto theta = theta0 - info_.beam_azimuth_angles[ipx];
      const auto signal = pf.px_signal(px_buf);  // u16

      // Image (range, azimuth, intensity), ignore ambient and reflectivity
      auto& v = image_.at<cv::Vec3f>(ipx, curr_col_);
      v[0] = range * valid;  // range or 0
      v[1] = theta;          // azimuth
      v[2] = signal;         // intensity

      // Cloud (see software manual figure 3.1)
      const auto n = info_.lidar_origin_to_beam_origin_mm * kMmToM;
      const auto d = range - n;
      const auto phi = info_.beam_altitude_angles[ipx];  // from high to low
      const auto cos_phi = std::cos(phi);

      // TODO (chao): handle invalid case for point
      auto& p = cloud_.at(curr_col_, ipx);  // (col, row)
      p.x = d * std::cos(theta) * cos_phi + n * std::cos(theta0);
      p.y = d * std::sin(theta) * cos_phi + n * std::sin(theta0);
      p.z = d * std::sin(phi);
      p.intensity = signal;
    }
  }
}

auto Decoder::DecodeImu(const uint8_t* const buf) -> sensor_msgs::Imu {
  sensor_msgs::Imu m;
  const auto& pf = *pf_;

  m.header.stamp.fromNSec(pf.imu_gyro_ts(buf));
  m.header.frame_id = imu_frame_;

  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 0;

  m.linear_acceleration.x = pf.imu_la_x(buf) * gravity_;
  m.linear_acceleration.y = pf.imu_la_y(buf) * gravity_;
  m.linear_acceleration.z = pf.imu_la_z(buf) * gravity_;

  m.angular_velocity.x = deg2rad(pf.imu_av_x(buf));
  m.angular_velocity.y = deg2rad(pf.imu_av_y(buf));
  m.angular_velocity.z = deg2rad(pf.imu_av_z(buf));

  for (int i = 0; i < 9; ++i) {
    m.orientation_covariance[i] = -1;
    m.angular_velocity_covariance[i] = 0;
    m.linear_acceleration_covariance[i] = 0;
  }
  for (int i = 0; i < 9; i += 4) {
    m.linear_acceleration_covariance[i] = 0.01;
    m.angular_velocity_covariance[i] = 6e-4;
  }

  return m;
}

}  // namespace ouster_decoder

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_decoder");

  ouster_decoder::Decoder node(ros::NodeHandle("~"));
  ros::spin();

  return 0;
}
