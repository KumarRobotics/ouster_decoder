#include "ouster_decoder/model.h"

#include <gtest/gtest.h>

#include <opencv2/core/mat.hpp>

namespace {

using ouster_decoder::LidarData;

TEST(UtilsTest, TestEncode) {
  cv::Mat image = cv::Mat(1, 1, CV_32FC4);
  auto& data = image.at<LidarData>(0, 0);

  data.range = 1.0;
  data.theta = 2.0;
  data.col = 3;
  data.intensity = 4;
  data.ambient = 5;
  data.reflectivity = 6;

  const auto& v = image.at<cv::Vec4f>(0, 0);
  EXPECT_EQ(v[0], 1.0);
  EXPECT_EQ(v[1], 2.0);

  const auto& d = image.at<LidarData>(0, 0);
  EXPECT_EQ(d.range, 1.0);
  EXPECT_EQ(d.theta, 2.0);
  EXPECT_EQ(d.col, 3);
  EXPECT_EQ(d.intensity, 4);
}

}  // namespace