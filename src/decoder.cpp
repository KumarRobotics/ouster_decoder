#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>

#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/ros.h"

namespace ouster_decoder {

namespace {

constexpr float kFloatInf = std::numeric_limits<float>::infinity();
constexpr float kFloatNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kPi = M_PI;
constexpr double kTau = 2 * kPi;
constexpr double kMmToM = 0.001f;
constexpr double kDefaultGravity = 9.807;  // [m/s^2] earth gravity
constexpr double Deg2Rad(double deg) { return deg * kPi / 180.0; }
template <typename T>
constexpr T Clamp(const T& v, const T& lo, const T& hi) {
  return std::max(lo, std::min(v, hi));
}

// Convert a vector of double from deg to rad
std::vector<double> TransformDeg2Rad(const std::vector<double>& degs) {
  std::vector<double> rads;
  rads.reserve(degs.size());
  for (const auto& deg : degs) {
    rads.push_back(Deg2Rad(deg));
  }
  return rads;
}

}  // namespace

namespace os = ouster_ros::sensor;

/// @brief stores decoded lidar data from a packet
struct LidarData {
  float range{};            // range in meter
  float theta{};            // azimuth angle
  uint16_t shift{};         // pixel shift
  uint16_t signal{};        // signal intensity photon
  uint16_t ambient{};       // NIR photons
  uint16_t reflectivity{};  // calibrated reflectivity
} __attribute__((packed));

static_assert(sizeof(LidarData) == sizeof(float) * 4,
              "Size of LidarData must be 4 floats (16 bytes)");

/// @brief stores lidar sensor info
struct LidarModel {
  LidarModel() = default;
  explicit LidarModel(const os::sensor_info& info);

  int rows{};                     // number of beams
  int cols{};                     // cols of a full scan
  int freq{};                     // frequency
  double dt_col{};                // delta time between two columns [s]
  double d_azimuth{};             // delta angle between two columns [rad]
  double beam_offset{};           // distance between beam to origin
  std::vector<double> azimuths;   // azimuths offset angles [rad]
  std::vector<double> altitudes;  // altitude angles, high to low [rad]
  std::vector<int> pixel_shifts;  // offset pixel count
  std::string prod_line;          // produnction line

  void ToPoint(Eigen::Ref<Eigen::Array3f> pt,
               float range,
               float theta0,
               int row);
  // Return a unique id of this col measurement
  int Uid(int fid, int mid) const noexcept { return fid * cols + mid; }
  void ToCameraInfo(sensor_msgs::CameraInfo& cinfo);
};

LidarModel::LidarModel(const os::sensor_info& info) {
  rows = info.beam_altitude_angles.size();
  cols = os::n_cols_of_lidar_mode(info.mode);
  freq = os::frequency_of_lidar_mode(info.mode);

  dt_col = 1.0 / freq / cols;
  d_azimuth = kTau / cols;
  beam_offset = info.lidar_origin_to_beam_origin_mm * kMmToM;
  pixel_shifts = info.format.pixel_shift_by_row;
  altitudes = TransformDeg2Rad(info.beam_altitude_angles);
  azimuths = TransformDeg2Rad(info.beam_azimuth_angles);

  prod_line = info.prod_line;
}

void LidarModel::ToPoint(Eigen::Ref<Eigen::Array3f> pt,
                         float range,
                         float theta0,
                         int row) {
  const float n = beam_offset;
  const float d = range - n;
  const float phi = altitudes[row];
  const float cos_phi = std::cos(phi);
  const float theta = theta0 - azimuths[row];

  pt.x() = d * std::cos(theta) * cos_phi + n * std::cos(theta0);
  pt.y() = d * std::sin(theta) * cos_phi + n * std::sin(theta0);
  pt.z() = d * std::sin(phi);
}

void LidarModel::ToCameraInfo(sensor_msgs::CameraInfo& cinfo) {
  cinfo.height = rows;
  cinfo.width = cols;
  cinfo.distortion_model = prod_line;

  cinfo.D.reserve(altitudes.size() + azimuths.size());
  cinfo.D.insert(cinfo.D.end(), altitudes.begin(), altitudes.end());
  cinfo.D.insert(cinfo.D.end(), azimuths.begin(), azimuths.end());

  cinfo.K[0] = dt_col;
  cinfo.R[0] = d_azimuth;
  cinfo.P[0] = beam_offset;
}

// TODO: move some stuff to this class
struct LidarScan {};

class Decoder {
 public:
  using PointT = pcl::PointXYZRGB;
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
  /// Decode one column in the lidar packet
  void DecodeColumn(const uint8_t* const col_buf);
  /// Zero out the current column in the cloud
  void ZeroCloudColumn(int col);
  /// Verify incoming data
  void VerifyData(int fid, int mid);
  /// Whether we are still waiting for alignment to mid 0
  bool CheckAlign(int mid);

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
  LidarModel model_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;
  cv::Mat image_;
  CloudT cloud_;
  std::vector<uint64_t> timestamps_;  // all time stamps (nanosecond)

  // params
  bool align_{};        // whether to align scan
  bool destagger_{};    // destagger image
  int curr_col_{0};     // current column
  int curr_scan_{0};    // current subscan
  double gravity_{};    // gravity
  double dt_packet_{};  // time between two packets
};

// Decoder
// Calls os_config service to get sensor info.
// Subscribes to lidar and imu packets and publishes image, camera info, point
// cloud, imu and static transforms
Decoder::Decoder(const ros::NodeHandle& pnh) : pnh_(pnh), it_(pnh) {
  InitOuster();
  InitParams();
  InitRos();
  SendTransform();
}

void Decoder::InitOuster() {
  // Call service to retrieve sensor info, this must be done first
  ouster_ros::OSConfigSrv cfg{};
  auto client = pnh_.serviceClient<ouster_ros::OSConfigSrv>("os_config");
  client.waitForExistence();
  if (!client.call(cfg)) {
    throw std::runtime_error("Calling config service failed");
  }

  // Parsing config
  info_ = os::parse_metadata(cfg.response.metadata);
  // ROS_DEBUG_STREAM("Metadata: " << os::to_string(info_));
  ROS_INFO_STREAM("Lidar mode: " << os::to_string(info_.mode));

  // Get packet format
  pf_ = &os::get_format(info_);
  ROS_INFO("Columns per packet: %d", pf_->columns_per_packet);
  ROS_INFO("Pixels per column: %d", pf_->pixels_per_column);
}

void Decoder::InitParams() {
  align_ = pnh_.param<bool>("align", false);
  gravity_ = pnh_.param<double>("gravity", kDefaultGravity);
  destagger_ = pnh_.param<bool>("destagger", false);

  // Div can only be 0,1,2, which means Subscan can only be 1,2,4
  int ndiv = pnh_.param<int>("ndiv", 0);
  ndiv = Clamp(ndiv, 0, 3);
  const int num_subscans = std::pow(2, ndiv);
  // Update destagger
  if (num_subscans != 1) {
    ROS_WARN("Divide full scan into %d subscans", num_subscans);
    // Destagger is disabled if we have more than one subscan
    destagger_ = false;
  }
  ROS_INFO("Destagger: %s", destagger_ ? "true" : "false");
  ROS_INFO("Gravity: %f", gravity_);
  ROS_INFO("Align: %s", align_ ? "true" : "false");

  // Model
  model_ = LidarModel(info_);
  ROS_INFO("Lidar: %d x %d @ %d hz", model_.rows, model_.cols, model_.freq);

  // Cinfo
  cinfo_msg_ = boost::make_shared<sensor_msgs::CameraInfo>();
  model_.ToCameraInfo(*cinfo_msg_);

  const int scan_cols = model_.cols / num_subscans;
  ROS_INFO("Subscan %d x %d", model_.rows, scan_cols);
  Allocate(model_.rows, scan_cols);

  // For timing purpose, time between each packet (multiple columns)
  // Ideally each callback should finish within this amount of time
  dt_packet_ = model_.dt_col * pf_->columns_per_packet;
}

void Decoder::InitRos() {
  // Subscribers
  lidar_sub_ =
      pnh_.subscribe("lidar_packets", 64, &Decoder::LidarPacketCb, this);
  imu_sub_ = pnh_.subscribe("imu_packets", 100, &Decoder::ImuPacketCb, this);
  ROS_INFO_STREAM("Subscribing lidar from: " << lidar_sub_.getTopic());
  ROS_INFO_STREAM("Subscribing imu from: " << imu_sub_.getTopic());

  // Publishers
  cloud_pub_ = pnh_.advertise<CloudT>("cloud", 10);
  camera_pub_ = it_.advertiseCamera("image", 5);
  imu_pub_ = pnh_.advertise<sensor_msgs::Imu>("imu", 100);
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
      info_.imu_to_sensor_transform, sensor_frame_, imu_frame_));

  static_tf_.sendTransform(ouster_ros::transform_to_tf_msg(
      info_.lidar_to_sensor_transform, sensor_frame_, lidar_frame_));
}

void Decoder::Reset() {
  // Reset curr_col (usually to 0 but in the rare case that data jumps forward
  // it will be non-zero)
  curr_col_ = curr_col_ % image_.cols;

  // Reset curr_scan if we have a full sweep
  if (curr_scan_ * image_.cols >= model_.cols) {
    curr_scan_ = 0;
  }

  // Zero out image in destagger mode
  if (destagger_) {
    image_.setTo(cv::Scalar());
  }
}

void Decoder::Allocate(int rows, int cols) {
  image_.create(rows, cols, CV_32FC4);
  cloud_ = CloudT(cols, rows);  // point cloud ctor takes width and height
  // cloud_ = CloudT(rows, cols);  // point cloud ctor takes width and height
  timestamps_.resize(cols, 0);
}

void Decoder::LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg) {
  const auto start = ros::Time::now();
  DecodeLidar(lidar_msg.buf.data());

  // We have data for the entire range image
  if (ShouldPublish()) {
    Publish();
    Timing(start);
  }
}

void Decoder::ImuPacketCb(const ouster_ros::PacketMsg& imu_msg) {
  imu_pub_.publish(DecodeImu(imu_msg.buf.data()));
}

void Decoder::Publish() {
  std_msgs::Header header;
  header.frame_id = lidar_frame_;
  header.stamp.fromNSec(timestamps_.front());

  // Publish image and camera_info
  // cinfo stores information about the full sweep, while roi stores information
  // about the subscan
  auto image_msg = cv_bridge::CvImage(header, "32FC4", image_).toImageMsg();
  cinfo_msg_->header = header;
  // Update camera info roi with curr_scan
  cinfo_msg_->roi.x_offset = curr_scan_ * image_.cols;
  cinfo_msg_->roi.y_offset = 0;
  cinfo_msg_->roi.width = image_.cols;
  cinfo_msg_->roi.height = image_.rows;
  cinfo_msg_->roi.do_rectify = destagger_;
  camera_pub_.publish(image_msg, cinfo_msg_);
  // Increment curr_scan
  ++curr_scan_;

  // Publish range image on demand
  if (range_pub_.getNumSubscribers() > 0) {
    cv::Mat range;
    cv::extractChannel(image_, range, 0);
    range_pub_.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());
  }

  // Publish cloud
  pcl_conversions::toPCL(header, cloud_.header);
  cloud_pub_.publish(cloud_);

  // Reset cached data after publish
  Reset();
}

void Decoder::Timing(const ros::Time& start) const {
  const auto t_end = ros::Time::now();
  const auto t_proc = (t_end - start).toSec();
  const auto ratio = t_proc / dt_packet_;
  if (ratio > 5) {
    ROS_WARN("Proc time: %f ms, meas time: %f ms, ratio: %f",
             t_proc * 1e3,
             dt_packet_ * 1e3,
             ratio);
  }
  ROS_DEBUG_THROTTLE(1,
                     "Proc time: %f ms, meas time: %f ms, ratio: %f",
                     t_proc * 1e3,
                     dt_packet_ * 1e3,
                     ratio);
}

bool Decoder::CheckAlign(int mid) {
  if (align_ && mid == 0) {
    align_ = false;
    ROS_DEBUG("Align start of the first subscan to mid 0");
  }
  return align_;
}

void Decoder::VerifyData(int fid, int mid) {
  static int prev_mid = -1;
  static int prev_fid = -1;

  if (prev_mid >= 0 && prev_fid >= 0) {
    // Compute uid of the current and previous measurement
    const int uid = model_.Uid(fid, mid);
    const int prev_uid = model_.Uid(prev_fid, prev_mid);

    // Ideally the jump should be 0
    const int jump = uid - prev_uid - 1;
    if (jump < 0) {
      ROS_FATAL(
          "Packet jumped from f%d:m%d to f%d:m%d by %d columns, which is "
          "backwards in time, shutting down.",
          prev_fid,
          prev_mid,
          fid,
          mid,
          jump);
      ros::shutdown();
    } else if (jump > 0) {
      ROS_ERROR("Packet jumped from f%d:m%d to f%d:m%d by %d columns.",
                prev_fid,
                prev_mid,
                fid,
                mid,
                jump);

      const auto start = ros::Time::now();
      // Detect a jump, we need to forward curr_col_ by the same amount as jump
      // We could directly increment curr_col_ and publish if necessary, but
      // this will require us to zero the whole cloud at publish time which is
      // very time consuming. Therefore, we choose to advance curr_col_ slowly
      // and zero out each column in the point cloud (no need to do it for the
      // image, because it is always zeroed on publish.
      for (int i = 0; i < jump; ++i) {
        // zero cloud column at curr_col_ and then increment
        ZeroCloudColumn(curr_col_++);

        // It is possible that this jump will span two scans, so if that is
        // the case, we need to publish the previous scan before moving forward
        if (ShouldPublish()) {
          ROS_DEBUG("Jumped into a new scan, need to publish the previous one");
          Publish();
        }
      }

      Timing(start);
    }
  }

  prev_mid = mid;
  prev_fid = fid;
}

void Decoder::DecodeLidar(const uint8_t* const packet_buf) {
  const auto& pf = *pf_;

  for (int icol = 0; icol < pf.columns_per_packet; ++icol) {
    // Get all the relevant data
    const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
    const auto fid = static_cast<int>(pf.col_frame_id(col_buf));
    const auto mid = static_cast<int>(pf.col_measurement_id(col_buf));

    // If we set the align param to true then this will wait for mid = 0 to
    // start a scan
    if (CheckAlign(mid)) {
      continue;
    }

    // Sometimes the lidar packet will jump forward by a large chunk, we handle
    // this case here
    VerifyData(fid, mid);
    // CHECK_LT(curr_col_, image_.cols);
    DecodeColumn(col_buf);

    // increment at last since we have a continue before
    ++curr_col_;
  }
}

void Decoder::ZeroCloudColumn(int col) {
  for (int ipx = 0; ipx < cloud_.height; ++ipx) {
    auto& pt = cloud_.at(col, ipx);
    pt.x = pt.y = pt.z = kFloatNaN;
  }
  // It is possible that the jump spans two subscans, this will cause the first
  // timestamp to be wrong when we publish the data, therefore we need to
  // extrapolate timestamp here
  timestamps_.at(col) =
      (col == 0 ? timestamps_.back() : timestamps_.at(col - 1)) +
      static_cast<uint64_t>(model_.dt_col * 1e9);
}

void Decoder::DecodeColumn(const uint8_t* const col_buf) {
  const auto& pf = *pf_;

  const uint64_t t_ns = pf.col_timestamp(col_buf);
  const uint32_t encoder = pf.col_encoder(col_buf);
  const uint32_t status = pf.col_status(col_buf);
  const bool col_valid = (status == 0xffffffff);

  // Compute azimuth angle theta0, this should always be valid
  // const auto theta0 = kTau - mid * model_.d_azimuth;
  const float theta0 = kTau * (1.0f - encoder / 90112.0f);
  // TODO (chao): if we have missing packets then the first time stamp could be
  // wrong
  timestamps_.at(curr_col_) = t_ns;

  for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
    const uint8_t* px_buf = pf.nth_px(ipx, col_buf);

    // im_col is where the pixel should go in the image
    // its the same as curr_col when we are in staggered mode
    const auto shift = model_.pixel_shifts[ipx];
    const int im_col =
        destagger_ ? (curr_col_ + shift) % image_.cols : curr_col_;

    const uint32_t raw_range = pf.px_range(px_buf);
    const float range = raw_range * kMmToM;
    const float theta = theta0 - model_.azimuths[ipx];

    auto& px = image_.at<LidarData>(ipx, im_col);
    px.range = col_valid ? range : kFloatNaN;      // 32
    px.theta = theta;                              // 32
    px.shift = shift;                              // 16
    px.signal = pf.px_signal(px_buf);              // 16
    px.ambient = pf.px_ambient(px_buf);            // 16
    px.reflectivity = pf.px_reflectivity(px_buf);  // 16

    // For point cloud we always publish staggered
    // all points in one column have the same time stamp
    // because we can always compute the range image based on xyz
    // https://www.ros.org/reps/rep-0117.html
    auto& pt = cloud_.at(curr_col_, ipx);  // (col, row)
    // pt.label = shift;  // can save 4 bytes if remove this field
    if (col_valid && range > 0.25) {
      model_.ToPoint(pt.getArray3fMap(), range, theta0, ipx);
      // TODO (chao): these magic numbers might need to be adjusted
      pt.r = std::min<uint16_t>(px.reflectivity / 32, 255);
      pt.g = std::min<uint16_t>(px.signal / 8, 255);
      pt.b = std::min<uint16_t>(px.ambient, 255);
      pt.a = 255;
    } else {
      // Invalid data, set to NaN
      pt.x = pt.y = pt.z = kFloatNaN;
    }
  }
}

auto Decoder::DecodeImu(const uint8_t* const buf) -> sensor_msgs::Imu {
  const auto& pf = *pf_;

  sensor_msgs::Imu m;
  m.header.stamp.fromNSec(pf.imu_gyro_ts(buf));
  m.header.frame_id = imu_frame_;

  m.orientation.x = 0;
  m.orientation.y = 0;
  m.orientation.z = 0;
  m.orientation.w = 0;

  m.linear_acceleration.x = static_cast<double>(pf.imu_la_x(buf)) * gravity_;
  m.linear_acceleration.y = static_cast<double>(pf.imu_la_y(buf)) * gravity_;
  m.linear_acceleration.z = static_cast<double>(pf.imu_la_z(buf)) * gravity_;

  m.angular_velocity.x = Deg2Rad(pf.imu_av_x(buf));
  m.angular_velocity.y = Deg2Rad(pf.imu_av_y(buf));
  m.angular_velocity.z = Deg2Rad(pf.imu_av_z(buf));

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
