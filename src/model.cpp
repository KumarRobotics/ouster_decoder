#include "ouster_decoder/model.h"

namespace ouster_decoder {

void LidarModel::ToCameraInfo(sensor_msgs::CameraInfo& cinfo) {
  cinfo.D.reserve(altitudes.size() + azimuths.size());
  cinfo.D.insert(cinfo.D.end(), altitudes.begin(), altitudes.end());
  cinfo.D.insert(cinfo.D.end(), azimuths.begin(), azimuths.end());

  cinfo.K[0] = dt_meas;    // delta time between two columns
  cinfo.R[0] = d_azimuth;  // delta angle between two columns
  cinfo.P[0] = beam_offset;
}

}  // namespace ouster_decoder
