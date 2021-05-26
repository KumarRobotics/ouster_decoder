#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbb/parallel_for.h>

#include <boost/timer/timer.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <string_view>

using PointT = pcl::PointXYZRGBL;
using CloudT = pcl::PointCloud<PointT>;

ros::Publisher pub1, pub2;
boost::timer::cpu_timer timer;

struct AngularRange {
  AngularRange() = default;
  AngularRange(float min, float max, int size)
      : min(min), max(max), size(size), delta((max - min) / size) {}

  float min;
  float max;
  float delta;
  int size;
};

template <typename Point>
float PointRange(const Point& p) {
  return std::hypot(p.x, p.y, p.z);
}

void Timing(const std::string& msg, const boost::timer::cpu_times& t) {
  ROS_INFO("%s %f ms", msg.c_str(), t.wall / 1e6);
}

void CloudCb(const CloudT& cloud) {
  cv::Mat image1 = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  cv::Mat image2 = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});

  // timer.start();
  // for (int i = 0; i < cloud.height; ++i) {
  //   for (int j = 0; j < cloud.width; ++j) {
  //     const auto& p = cloud.at(j, i);
  //     int col = (j + p.label) % cloud.width;
  //     image1.at<float>(i, col) = PointRange(p);
  //   }
  // }
  // Timing("copy to image for", timer.elapsed());

  // 4 times faster than serial
  timer.start();
  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.height),
                    [&image1, &cloud](const tbb::blocked_range<int>& range) {
                      for (int i = range.begin(); i < range.end(); ++i) {
                        for (int j = 0; j < cloud.width; ++j) {
                          const auto& p = cloud.at(j, i);
                          int col = (j + p.label) % cloud.width;
                          // int col = j + p.label;
                          // if (col < 0 || col >= cloud.width) continue;
                          image1.at<float>(i, col) = PointRange(p);
                        }
                      }
                    });
  Timing("copy to image par", timer.elapsed());

  // Convert to range image
  // first need to compute the delta azimuth
  const auto d_azimuth = M_PI * 2 / cloud.width;

  timer.start();
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& p = cloud.at(j, i);
      const auto range = PointRange(p);
      if (std::isnan(p.x)) continue;
      auto theta = std::atan2(p.y, p.x);
      theta = theta > 0 ? M_PI * 2 - theta : -theta;
      int col = static_cast<int>(theta / d_azimuth + 0.5);
      image2.at<float>(i, col) = PointRange(p);
    }
  }
  Timing("To range image for", timer.elapsed());

  std_msgs::Header header;
  pcl_conversions::fromPCL(cloud.header, header);
  pub1.publish(cv_bridge::CvImage(header, "32FC1", image1).toImageMsg());
  pub2.publish(cv_bridge::CvImage(header, "32FC1", image2).toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_example");

  ros::NodeHandle pnh("~");
  pub1 = pnh.advertise<sensor_msgs::Image>("range1", 1);
  pub2 = pnh.advertise<sensor_msgs::Image>("range2", 1);
  ros::Subscriber sub = pnh.subscribe("cloud", 5, &CloudCb);

  ros::spin();

  return 0;
}