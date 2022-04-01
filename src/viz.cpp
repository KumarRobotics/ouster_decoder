#include <cv_bridge/cv_bridge.h>
#include <fmt/core.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace ouster_decoder {

namespace it = image_transport;
namespace sm = sensor_msgs;

cv::Mat ApplyCmap(const cv::Mat& input, int cmap, uint8_t bad_color) {
  cv::Mat disp;
  input.convertTo(disp, CV_8UC1);
  cv::applyColorMap(disp, disp, cmap);
  return disp;
}

class Viz {
 public:
  explicit Viz(const ros::NodeHandle& pnh);

  void CameraCb(const sm::ImageConstPtr& image_ptr,
                const sm::CameraInfoConstPtr& cinfo_ptr);

 private:
  ros::NodeHandle pnh_;
  it::ImageTransport it_;
  it::CameraSubscriber sub_camera_;
  std::string cmap_{"gray"};
};

Viz::Viz(const ros::NodeHandle& pnh) : pnh_{pnh}, it_{pnh} {
  sub_camera_ = it_.subscribeCamera("image", 1, &Viz::CameraCb, this);
  ROS_INFO_STREAM("Subscribing to: " << sub_camera_.getTopic());
}

void Viz::CameraCb(const sm::ImageConstPtr& image_ptr,
                   const sm::CameraInfoConstPtr& cinfo_ptr) {
  const auto mat = cv_bridge::toCvShare(image_ptr)->image;
  const auto h = mat.rows;
  const auto w = mat.cols;

  // Extract range and intensity
  const cv::Mat mat_map(mat.rows, mat.cols, CV_16UC(8), mat.data);
  cv::Mat range_raw, signal_raw;
  cv::extractChannel(mat_map, range_raw, 6);
  cv::extractChannel(mat_map, signal_raw, 7);

  auto range_color = ApplyCmap(range_raw / 100, cv::COLORMAP_PINK, 0);
  auto signal_color = ApplyCmap(signal_raw / 4, cv::COLORMAP_PINK, 0);

  // set invalid range (0) to black
  range_color.setTo(0, range_raw == 0);
  signal_color.setTo(0, range_raw == 0);

  cv::Mat signal_half0;
  cv::Mat signal_half1;
  signal_half0.create(h / 2, w, CV_8UC3);
  signal_half1.create(h / 2, w, CV_8UC3);

  for (int r = 0; r < h; ++r) {
    const int rr = r / 2;
    if (r % 2 == 0) {
      signal_color.row(r).copyTo(signal_half0.row(rr));
    } else {
      signal_color.row(r).copyTo(signal_half1.row(rr));
    }
  }

  // save raw data to ouster
  const int id = image_ptr->header.seq;
  const std::vector<int> png_flags = {cv::IMWRITE_PNG_COMPRESSION, 0};
  cv::imwrite(fmt::format("/tmp/ouster/range_{:04d}.png", id), range_raw, png_flags);
  cv::imwrite(fmt::format("/tmp/ouster/signal_{:04d}.png", id), signal_raw, png_flags);
  ROS_INFO_STREAM("Writing message with id: " << id);

  constexpr auto win_flags =
      cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED | cv::WINDOW_NORMAL;

  cv::namedWindow("range", win_flags);
  cv::imshow("range", range_color);

  cv::namedWindow("signal", win_flags);
  cv::imshow("signal", signal_color);

  cv::namedWindow("signal_half0", win_flags);
  cv::imshow("signal_half0", signal_half0);

  cv::namedWindow("signal_half1", win_flags);
  cv::imshow("signal_half1", signal_half1);

  cv::waitKey(1);
}

}  // namespace ouster_decoder

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_viz");

  ouster_decoder::Viz node(ros::NodeHandle("~"));
  ros::spin();

  return 0;
}
