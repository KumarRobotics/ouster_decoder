#pragma once

#include <sensor_msgs/CameraInfo.h>

#include <cmath>
#include <cstdint>

namespace ouster_decoder {

struct LidarData {
  float range{};   // range in meter
  float theta{};   // azimuth angle
  uint16_t icol{};  // col in image, used to compute time of this data
  uint16_t intensity{};
  uint16_t ambient{};
  uint16_t reflectivity{};
} __attribute__((packed));

static_assert(sizeof(LidarData) == sizeof(float) * 4,
              "Size of Data must be 4 floats (16 bytes)");

struct LidarModel {
  double dt_meas;
  double d_azimuth;
  double beam_offset;
  std::vector<double> azimuths;
  std::vector<double> altitudes;
  std::vector<int> pixel_shifts;

  void ToCameraInfo(sensor_msgs::CameraInfo& cinfo);
  bool FromCameraInfo(const sensor_msgs::CameraInfo& cinfo);
};

}  // namespace ouster_decoder
