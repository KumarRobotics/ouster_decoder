#include <cv_bridge/cv_bridge.h>
#include <ouster_ros/point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbb/parallel_for.h>

#include <boost/timer/timer.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <string_view>

using PointMe = pcl::PointXYZRGBL;
using CloudMe = pcl::PointCloud<PointMe>;

using PointOs = ouster_ros::Point;
using CloudOs = pcl::PointCloud<PointOs>;

ros::Publisher pub_me_shift, pub_me_range, pub_os_range;
boost::timer::cpu_timer timer;

template <typename Point>
float PointRange(const Point& p) {
  return std::hypot(p.x, p.y, p.z);
}

void Timing(const std::string& msg, const boost::timer::cpu_times& t) {
  ROS_INFO("%s %f ms", msg.c_str(), t.wall / 1e6);
}

template <typename Point>
void FillPixel(float* row, const Point& p, float d_theta) {
  if (std::isnan(p.x)) return;

  const auto range = PointRange(p);
  float theta = std::atan2(p.y, p.x);
  theta = theta > 0.0 ? M_PI * 2 - theta : -theta;
  const int col = static_cast<int>(theta / d_theta);
  row[col] = PointRange(p);
}

template <typename T>
void CloudToRange(const pcl::PointCloud<T>& cloud, cv::Mat& image) {
  const float d_theta = M_PI * 2 / cloud.width;

  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      FillPixel(image.ptr<float>(i), cloud.at(j, i), d_theta);
    }
  }
}

template <typename T>
void CloudToRangePar(const pcl::PointCloud<T>& cloud, cv::Mat& image) {
  const float d_theta = M_PI * 2 / cloud.width;

  tbb::parallel_for(tbb::blocked_range<int>(0, cloud.height),
                    [&](const tbb::blocked_range<int>& br) {
                      for (int i = br.begin(); i < br.end(); ++i) {
                        for (int j = 0; j < cloud.width; ++j) {
                          FillPixel(
                              image.ptr<float>(i), cloud.at(j, i), d_theta);
                        }
                      }
                    });
}

// Ouster original
void CloudOsCb(const CloudOs& cloud) {
  cv::Mat range = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  timer.start();
  CloudToRangePar(cloud, range);
  Timing("Os cloud to range: ", timer.elapsed());

  std_msgs::Header header;
  pcl_conversions::fromPCL(cloud.header, header);
  pub_os_range.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());
}

void CloudMeCb(const CloudMe& cloud) {
  cv::Mat range = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  timer.start();
  CloudToRangePar(cloud, range);
  Timing("Me cloud to range: ", timer.elapsed());

  cv::Mat shift = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  // ouster img_node use the first pixel in the first row as the start of
  // the image, therefore we need subtract this offset
  timer.start();
  const int offset = cloud.front().label;
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& p = cloud.at(j, i);
      if (std::isnan(p.x)) continue;
      const int col = (j + p.label - offset) % cloud.width;
      shift.at<float>(i, col) = PointRange(p);
    }
  }
  Timing("Me cloud to shift: ", timer.elapsed());

  std_msgs::Header header;
  pcl_conversions::fromPCL(cloud.header, header);
  pub_me_range.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());
  pub_me_shift.publish(cv_bridge::CvImage(header, "32FC1", shift).toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_example");

  ros::NodeHandle pnh("~");
  ros::Subscriber sub_os = pnh.subscribe("cloud_os", 5, &CloudOsCb);
  ros::Subscriber sub_me = pnh.subscribe("cloud_me", 5, &CloudMeCb);

  pub_me_shift = pnh.advertise<sensor_msgs::Image>("shift_me", 1);
  pub_me_range = pnh.advertise<sensor_msgs::Image>("range_me", 1);
  pub_os_range = pnh.advertise<sensor_msgs::Image>("range_os", 1);

  ros::spin();

  return 0;
}