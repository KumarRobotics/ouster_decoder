#include "lidar.h"

#include <boost/make_shared.hpp>

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
  // cinfo.K[1] = d_azimuth;    // radian between each column
  // cinfo.K[2] = beam_offset;  // distance from center to beam
}

void LidarScan::Allocate(int rows, int cols) {
  // Don't do any work if rows and cols are the same
  //  image.create(rows, cols, CV_32FC4);
  if (!image_ptr) image_ptr = boost::make_shared<sensor_msgs::Image>();

  image_ptr->height = rows;
  image_ptr->width = cols;
  image_ptr->encoding = "32FC4";
  image_ptr->step = cols * sizeof(ImageData);
  image_ptr->data.resize(rows * cols * sizeof(ImageData));

  cloud.width = cols;
  cloud.height = rows;
  cloud.point_step = 16;  // xyzi
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields = MakePointFieldsXYZI();
  cloud.is_dense = 1u;
  cloud.data.resize(rows * cols * cloud.point_step);

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
}

void LidarScan::SoftReset(int full_col) noexcept {
  // Reset col (usually to 0 but in the rare case that data jumps forward
  // it will be non-zero)
  icol = icol % cols();

  // Reset scan if we have a full sweep
  if (iscan * cols() >= full_col) {
    iscan = 0;
  }
}

void LidarScan::InvalidateColumn(double dt_col) noexcept {
  for (int irow = 0; irow < static_cast<int>(cloud.height); ++irow) {
    auto* ptr = CloudPtr(irow, icol);
    ptr[0] = ptr[1] = ptr[2] = kNaNF;
  }

  for (int irow = 0; irow < rows(); ++irow) {
    auto* ptr = ImagePtr(irow, icol);
    ptr->set_bad();
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
  const uint16_t mid = pf.col_measurement_id(col_buf);
  const uint32_t status = pf.col_status(col_buf) & 0x0001;
  // Legacy mode
  // const bool col_valid = (status == 0xffffffff);
  //  const bool col_valid = status;

  // Compute azimuth angle theta0, this should always be valid
  // const auto theta_enc = kTau - mid * model_.d_azimuth;
  // const float theta_enc = kTau * (1.0f - encoder / 90112.0f);
  const float theta_enc = kTau - mid * model.d_azimuth;
  times.at(icol) = t_ns;

  for (int ipx = 0; ipx < pf.pixels_per_column; ++ipx) {
    // Data to fill
    Eigen::Vector3f xyz;
    xyz.setConstant(kNaNF);
    float r{};
    uint16_t s16u{};

    if (status) {
      const uint8_t* const px_buf = pf.nth_px(ipx, col_buf);
      const auto raw_range = pf.px_range(px_buf);
      const float range = raw_range * kMmToM;  // used to compute xyz

      if (min_range <= range && range <= max_range) {
        xyz = model.ToPoint(range, theta_enc, ipx);
        r = xyz.norm();  // we compute range ourselves
        s16u = pf.px_signal(px_buf);
      }
      // s16u += pf.px_ambient(px_buf);
    }

    // Now we set cloud and image data
    // There is no destagger for cloud, so we update point no matter what
    auto* cptr = CloudPtr(ipx, icol);
    cptr[0] = xyz.x();
    cptr[1] = xyz.y();
    cptr[2] = xyz.z();
    cptr[3] = static_cast<float>(s16u);

    // However image can be destaggered, and pixel can go out of bound
    // add pixel shift to get where the pixel should be
    const auto col_shift = model.pixel_shifts()[ipx];
    const auto im_col = destagger ? icol + col_shift : icol;

    if (0 <= im_col && im_col < cols()) {
      auto* iptr = ImagePtr(ipx, im_col);
      iptr->x = xyz.x();
      iptr->y = xyz.y();
      iptr->z = xyz.z();
      iptr->set_range(r, range_scale);
      iptr->s16u = s16u;
    } else {
      auto* iptr = ImagePtr(ipx, im_col % cols());
      iptr->set_bad();
    }
  }

  // Move on to next column
  ++icol;
}

void LidarScan::UpdateCinfo(sensor_msgs::CameraInfo& cinfo) const noexcept {
  cinfo.R[0] = range_scale;
  cinfo.binning_x = iscan;  // index of subscan within a full scan

  // Update camera info roi with curr_scan
  auto& roi = cinfo.roi;
  roi.x_offset = StartingCol();
  roi.y_offset = 0;  // this is always 0
  roi.width = cols();
  roi.height = rows();
  roi.do_rectify = destagger;
}

std::vector<sensor_msgs::PointField> MakePointFieldsXYZI() noexcept {
  using sensor_msgs::PointField;
  std::vector<PointField> fields;
  fields.reserve(4);

  PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  fields.push_back(field);

  field.name = "intensity";
  field.offset = 12;
  field.datatype = PointField::FLOAT32;
  field.count = 1;
  fields.push_back(field);

  return fields;
}

}  // namespace ouster_decoder
