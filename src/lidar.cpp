#include "lidar.h"

namespace ouster_decoder {

namespace os = ouster_ros::sensor;

constexpr float kMmToM = 0.001;
constexpr double kTau = 2 * M_PI;
constexpr float kNaNF = std::numeric_limits<float>::quiet_NaN();

namespace {

/// @brief Convert a vector of double from deg to rad
std::vector<double> TransformDeg2Rad(const std::vector<double>& degs) {
  std::vector<double> rads;
  rads.reserve(degs.size());
  for (const auto& deg : degs) {
    rads.push_back(Deg2Rad(deg));
  }
  return rads;
}

}  // namespace

LidarModel::LidarModel(const std::string& metadata) {
  info = os::parse_metadata(metadata);
  pf = &os::get_format(info);

  rows = info.beam_altitude_angles.size();
  cols = os::n_cols_of_lidar_mode(info.mode);
  freq = os::frequency_of_lidar_mode(info.mode);

  dt_col = 1.0 / freq / cols;
  d_azimuth = kTau / cols;
  dt_packet = dt_col * pf->columns_per_packet;
  beam_offset = info.lidar_origin_to_beam_origin_mm * kMmToM;
  altitudes = TransformDeg2Rad(info.beam_altitude_angles);
  azimuths = TransformDeg2Rad(info.beam_azimuth_angles);
}

Eigen::Vector3f LidarModel::ToPoint(float range,
                                    float theta_enc,
                                    int row) const {
  const float n = beam_offset;
  const float d = range - n;
  const float phi = altitudes[row];
  const float cos_phi = std::cos(phi);
  const float theta = theta_enc - azimuths[row];

  return {d * std::cos(theta) * cos_phi + n * std::cos(theta_enc),
          d * std::sin(theta) * cos_phi + n * std::sin(theta_enc),
          d * std::sin(phi)};
}

void LidarModel::UpdateCameraInfo(sensor_msgs::CameraInfo& cinfo) const {
  cinfo.height = rows;
  cinfo.width = cols;
  cinfo.distortion_model = info.prod_line;

  // cinfo.D.reserve(altitudes.size() + azimuths.size());
  // cinfo.D.insert(cinfo.D.end(), altitudes.begin(), altitudes.end());
  // cinfo.D.insert(cinfo.D.end(), azimuths.begin(), azimuths.end());
  cinfo.D.reserve(pixel_shifts().size());
  cinfo.D.insert(cinfo.D.end(), pixel_shifts().begin(), pixel_shifts().end());

  cinfo.K[0] = dt_col;  // time between each column
  //  cinfo.K[1] = d_azimuth;    // radian between each column
  //  cinfo.K[2] = beam_offset;  // distance from center to beam
}

void LidarScan::Allocate(int rows, int cols) {
  // Don't do any work if rows and cols are the same
  if (rows == image.rows && cols == image.cols) return;
  image.create(rows, cols, CV_32FC4);
  cloud = CloudT(cols, rows);  // point cloud ctor takes width and height
  times.clear();
  times.resize(cols, 0);
}

int LidarScan::DetectJump(int uid) noexcept {
  int jump = 0;

  if (prev_uid >= 0) {
    // Ideally the increment should be 1 hence the jump should be 0
    jump = uid - prev_uid - 1;
  }

  prev_uid = uid;
  return jump;
}

void LidarScan::HardReset() noexcept {
  icol = 0;
  iscan = 0;
  prev_uid = -1;
  num_valid = 0;
}

void LidarScan::SoftReset(int full_col) noexcept {
  num_valid = 0;
  // Reset col (usually to 0 but in the rare case that data jumps forward
  // it will be non-zero)
  icol = icol % image.cols;

  // Reset scan if we have a full sweep
  if (iscan * image.cols >= full_col) {
    iscan = 0;
  }
}

void LidarScan::InvalidateColumn(double dt_col) {
  for (int irow = 0; irow < static_cast<int>(cloud.height); ++irow) {
    auto& pt = cloud.at(icol, irow);
    pt.x = pt.y = pt.z = kNaNF;
  }

  for (int irow = 0; irow < image.rows; ++irow) {
    auto& px = image.at<cv::Vec4f>(irow, icol);
    px[0] = px[1] = px[2] = px[3] = kNaNF;
  }

  // It is possible that the jump spans two subscans, this will cause the
  // first timestamp to be wrong when we publish the data, therefore we need
  // to extrapolate timestamp here
  times.at(icol) = (icol == 0 ? times.back() : times.at(icol - 1)) +
                   static_cast<uint64_t>(dt_col * 1e9);

  // Move on to next column
  ++icol;
}

void LidarScan::DecodeColumn(const uint8_t* const col_buf,
                             const LidarModel& model) {
  const auto& pf = *model.pf;
  const uint64_t t_ns = pf.col_timestamp(col_buf);
  const uint32_t encoder = pf.col_encoder(col_buf);
  const uint32_t status = pf.col_status(col_buf);
  bool col_valid = (status == 0xffffffff);

  // Compute azimuth angle theta0, this should always be valid
  // const auto theta_enc = kTau - mid * model_.d_azimuth;
  const float theta_enc = kTau * (1.0f - encoder / 90112.0f);
  times.at(icol) = t_ns;

  for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
    const uint8_t* const px_buf = pf.nth_px(ipx, col_buf);
    const auto raw_range = pf.px_range(px_buf);
    const float range = raw_range * kMmToM;

    // im_col is where the pixel should go in the image
    // it is the same as icol when we are not in staggered mode
    int im_col = icol;
    if (destagger) {
      // add pixel shift to get where the pixel should be
      im_col += model.pixel_shifts()[ipx];
      // if it is outside the current subscan, we set this pixel invalid
      if (im_col < 0 || im_col >= cols()) {
        col_valid = false;
        // make sure index is within bound
        im_col = im_col % cols();
      }
    }

    // Set point
    auto& pt = cloud.at(icol, ipx);
    auto& px = image.at<ImageData>(ipx, im_col);
    if (col_valid && min_range < range && range < max_range) {
      pt.getVector3fMap() = model.ToPoint(range, theta_enc, ipx);

      // https://github.com/ouster-lidar/ouster_example/issues/128
      // Intensity: whereas most "normal" surfaces lie in between 0 - 1000

      // Consider Intensity-SLAM
      // https://arxiv.org/pdf/2102.03798.pdf
      const float r = pt.getVector3fMap().norm();
      px.r = static_cast<uint16_t>(std::min(r * range_scale, 65535.0));

      px.intensity = std::min(pf.px_reflectivity(px_buf) / 256, 255);
      // px.intensity = std::min(pf.px_signal(px_buf) * r * r / 256, 255.0f);
      // px.intensity = std::min(pf.px_signal(px_buf) / 4, 255);
      pt.intensity = static_cast<float>(px.intensity);
      ++num_valid;  // increment valid points
    } else {
      pt.x = pt.y = pt.z = pt.intensity = kNaNF;
      px.r = px.intensity = 0;
    }

    px.x = pt.x;
    px.y = pt.y;
    px.z = pt.z;
  }

  // Move on to next column
  ++icol;
}

void LidarScan::UpdateCinfo(sensor_msgs::CameraInfo& cinfo) const noexcept {
  cinfo.R[0] = range_scale;
  cinfo.binning_x = iscan;
  cinfo.binning_y = num_valid;

  // Update camera info roi with curr_scan
  auto& roi = cinfo.roi;
  roi.x_offset = StartingCol();
  roi.y_offset = 0;
  roi.width = image.cols;
  roi.height = image.rows;
  roi.do_rectify = destagger;
}

}  // namespace ouster_decoder
