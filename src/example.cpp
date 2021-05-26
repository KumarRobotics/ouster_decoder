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

template <typename T>
void CloudToRange(const pcl::PointCloud<T>& cloud, cv::Mat& image) {
  const float d_theta = M_PI * 2 / cloud.width;

  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& p = cloud.at(j, i);
      if (std::isnan(p.x)) continue;

      const auto range = PointRange(p);
      if (range < 0.5 || range > 100) continue;

      float theta = std::atan2(p.y, p.x);
      theta = theta > 0.0 ? M_PI * 2 - theta : -theta;
      const int col = static_cast<int>(theta / d_theta);
      image.at<float>(i, col) = PointRange(p);
    }
  }
}

// Ouster original
void CloudOsCb(const CloudOs& cloud) {
  cv::Mat range = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  CloudToRange(cloud, range);

  std_msgs::Header header;
  pcl_conversions::fromPCL(cloud.header, header);
  pub_os_range.publish(cv_bridge::CvImage(header, "32FC1", range).toImageMsg());
}

void CloudMeCb(const CloudMe& cloud) {
  cv::Mat range = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  CloudToRange(cloud, range);

  cv::Mat shift = cv::Mat(cloud.height, cloud.width, CV_32FC1, cv::Scalar{});
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& p = cloud.at(j, i);
      const int col = (j + p.label - 32) % cloud.width;
      if (std::isnan(p.x)) continue;
      shift.at<float>(i, col) = PointRange(p);
    }
  }

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