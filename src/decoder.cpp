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
  range_pub_ = pnh_.advertise<sensor_msgs::Image>("range", 1);

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

  // Model
  model_.dt_meas = 1.0 / freq / cols;
  model_.d_azimuth = kTau / cols;
  model_.beam_offset = info_.lidar_origin_to_beam_origin_mm * kMmToM;
  model_.altitudes = info_.beam_altitude_angles;
  model_.azimuths = info_.beam_azimuth_angles;
  TransformDeg2RadInPlace(model_.altitudes);
  TransformDeg2RadInPlace(model_.azimuths);

  // Cinfo
  cinfo_msg_ = boost::make_shared<sensor_msgs::CameraInfo>();
  cinfo_msg_->height = rows;
  cinfo_msg_->width = cols;
  cinfo_msg_->distortion_model = info_.prod_line;
  model_.ToCameraInfo(*cinfo_msg_);

  dt_packet_ = model_.dt_meas * pf_->columns_per_packet;  // dt two packet
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  ROS_INFO("Gravity: %f", gravity_);
  destagger_ = pnh_.param<bool>("destagger", false);
  ROS_INFO("Destagger: %s", destagger_ ? "true" : "false");

  Allocate(rows, cols);
  SendTransform();
}

void Decoder::SendTransform() {
  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      info_.imu_to_sensor_transform, sensor_frame_, imu_frame_));

  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      info_.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));
}

void Decoder::Reset() {
  curr_col_ = 0;
  // No need to zero out if we don't need to destagger
  if (!destagger_) return;
  // Otherwise we will zero out the cached data
  image_.setTo(cv::Scalar());
  // TODO (chao): set cloud_ to 0
}

void Decoder::Allocate(int rows, int cols) {
  image_.create(rows, cols, CV_32FC4);
  cloud_ = CloudT(cols, rows);  // point cloud ctor takes width and height
  timestamps_.resize(cols);
  azimuths_.resize(cols);
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto start = ros::Time::now();
  DecodeLidar(lidar_msg.buf.data());

  // We have data for the entire range image
  if (HaveFullScan()) {
    Publish();
    Reset();
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
  auto image_msg = cv_bridge::CvImage(header, "32FC4", image_).toImageMsg();
  cinfo_msg_->header = header;
  camera_pub_.publish(image_msg, cinfo_msg_);

  // Publish range image to test destagger
  cv::Mat range;
  cv::extractChannel(image_, range, 0);
  range_pub_.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());

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
    const uint32_t encoder = pf.col_encoder(col_buf);  // not necessary
    const uint32_t status = pf.col_status(col_buf);
    const bool valid = (status == 0xffffffff);

    // Compute azimuth angle theta0, this should always be valid
    // const auto theta0 = kTau - meas_id * model_.d_azimuth;
    const auto theta0 = kTau * (1.0 - encoder / 90112.0f);
    timestamps_.at(curr_col_) = t_ns;
    azimuths_.at(curr_col_) = theta0;

    for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
      const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

      const float range = pf.px_range(px_buf) * kMmToM;
      const float theta = theta0 - model_.azimuths[ipx];
      const auto signal = pf.px_signal(px_buf);  // u16

      auto& px = image_.at<LidarData>(ipx, curr_col_);
      px.range = range * valid;                      // 32
      px.theta = theta;                              // 32
      px.col = curr_col_;                            // 16
      px.intensity = pf.px_signal(px_buf);           // 16
      px.ambient = pf.px_ambient(px_buf);            // 16
      px.reflectivity = pf.px_reflectivity(px_buf);  // 16

      // Cloud (see software manual figure 3.1)
      const auto n = model_.beam_offset;
      const auto d = range - n;
      const auto phi = model_.altitudes[ipx];  // from high to low
      const auto cos_phi = std::cos(phi);

      // TODO (chao): handle invalid case for point
      auto& pt = cloud_.at(curr_col_, ipx);  // (col, row)
      pt.x = d * std::cos(theta) * cos_phi + n * std::cos(theta0);
      pt.y = d * std::sin(theta) * cos_phi + n * std::sin(theta0);
      pt.z = d * std::sin(phi);
      pt.data_c[0] = px.intensity;
      pt.data_c[1] = px.ambient;
      pt.data_c[2] = px.reflectivity;
      // pt.data_c[3] = meas_id;
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
