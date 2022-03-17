#include <cv_bridge/cv_bridge.h>
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

  // Extract range and intensity
  const cv::Mat mat_map(mat.rows, mat.cols, CV_16UC(8), mat.data);
  cv::Mat range_raw, signal_raw;
  cv::extractChannel(mat_map, range_raw, 6);
  cv::extractChannel(mat_map, signal_raw, 7);

  range_raw /= 100;
  signal_raw /= 2;

  auto signal_color = ApplyCmap(signal_raw, cv::COLORMAP_PINK, 0);
  auto range_color = ApplyCmap(range_raw, cv::COLORMAP_PINK, 0);

  // set invalid range (0) to black
  range_color.setTo(0, range_raw == 0);

  constexpr auto win_flags =
      cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED | cv::WINDOW_NORMAL;
  cv::namedWindow("range", win_flags);
  cv::imshow("range", range_color);
  cv::namedWindow("intensity", win_flags);
  cv::imshow("intensity", signal_color);
  cv::waitKey(1);
}

}  // namespace ouster_decoder

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_viz");

  ouster_decoder::Viz node(ros::NodeHandle("~"));
  ros::spin();

  return 0;
}
