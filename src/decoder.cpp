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
static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }
static constexpr double kPi = M_PI;
static constexpr double kTau = 2 * kPi;
static constexpr double kMmToM = 0.001f;

// Convert a vector of double from deg to rad
void TransformDeg2RadInPlace(std::vector<double>& vec) {
  std::transform(vec.begin(), vec.end(), vec.begin(), deg2rad);
}

}  // namespace

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
  lidar_frame_ = pnh_.param<std::string>("lidar_frame", "os_lidar");
  imu_frame_ = pnh_.param<std::string>("lidar_frame", "os_imu");
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

  dt_col_ = 1.0 / freq / cols;
  dt_packet_ = dt_col_ * pf_->columns_per_packet;  // time two packet
  d_azimuth_ = kTau / cols;                        // angle between two columns

  // Transform angle from degree to radian
  TransformDeg2RadInPlace(info_.beam_altitude_angles);
  TransformDeg2RadInPlace(info_.beam_azimuth_angles);

  // Allocate
  image_.create(rows, cols, CV_32FC3);
  cloud_ = CloudT(cols, rows);
  ts_.resize(cols);
  azimuths_.resize(cols);
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto t_beg = ros::Time::now();

  uint8_t const* const packet_buf = lidar_msg.buf.data();
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

    // Compute azimuth angle theta0
    const auto theta0 = kTau - meas_id * d_azimuth_;
    ts_[curr_col_] = t_ns;
    azimuths_[curr_col_] = theta0;

    for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
      const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

      const float range = pf.px_range(px_buf) * kMmToM * valid;
      const float theta = theta0 - info_.beam_azimuth_angles[ipx];
      const float signal = pf.px_signal(px_buf);

      // Image
      auto& v = image_.at<cv::Vec3f>(ipx, curr_col_);
      v[0] = range;   // range or 0
      v[1] = theta;   // azimuth
      v[2] = signal;  // intensity

      // Cloud
      const auto n = info_.lidar_origin_to_beam_origin_mm * kMmToM;
      const auto phi = info_.beam_altitude_angles[ipx];
      const float d = range - n;
      const auto cos_phi = std::cos(phi);

      auto& p = cloud_.at(curr_col_, ipx);
      p.x = d * std::cos(theta) * cos_phi + n * std::cos(theta0);
      p.y = d * std::sin(theta) * cos_phi + n * std::sin(theta0);
      p.z = d * std::sin(phi);
      p.intensity = signal;
    }
  }

  // We have data for the entire range image
  if (curr_col_ >= image_.cols) {
    curr_col_ = 0;

    std_msgs::Header header;
    header.frame_id = lidar_frame_;
    header.stamp.fromNSec(ts_.front());

    // Publish
    auto image_msg = cv_bridge::CvImage(header, "32FC3", image_).toImageMsg();
    auto cinfo_msg = boost::make_shared<sensor_msgs::CameraInfo>();
    camera_pub_.publish(image_msg, cinfo_msg);

    // Publish cloud
    pcl_conversions::toPCL(header, cloud_.header);
    cloud_pub_.publish(cloud_);
  }

  const auto t_end = ros::Time::now();
  const auto t_proc = (t_end - t_beg).toSec();
  const auto ratio = t_proc / dt_packet_;
  if (ratio > 1) {
    ROS_WARN("Proc time: %f us, meas time: %f us, ratio: %f",
             t_proc * 1e6,
             dt_packet_ * 1e6,
             ratio);
  }
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
