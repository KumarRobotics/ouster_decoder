#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbb/parallel_for.h>

#include <opencv2/core/mat.hpp>

ros::Publisher pub;
using PointT = pcl::PointXYZRGBL;
using CloudT = pcl::PointCloud<PointT>;

struct AngularRange {
  AngularRange() = default;
  AngularRange(float min, float max, int size)
      : min(min), max(max), size(size), delta((max - min) / size) {}

  float min;
  float max;
  float delta;
  int size;
};

void CloudCb2(const CloudT& cloud) {
  static cv::Mat image;

  const int im_rows = 128;
  const int im_cols = 1024;

  AngularRange phi_rg(-30 * M_PI / 180, 30 * M_PI / 180, im_rows);
  AngularRange theta_rg(0, 2 * M_PI, im_cols);

  if (image.empty()) {
    image.create(im_rows, im_cols, CV_16UC1);
  }

  // Convert to index image
  auto t0 = ros::Time::now();
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      int k = i * cloud.width + j;
      const auto& pt = cloud.at(j, i);
      // const auto jj = (j + pt.label) % cloud.width;
      // compute azimuth
      const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (r2 < 0.5 * 0.5) continue;
      const float phi = std::atan2(pt.z, std::hypot(pt.x, pt.y));
      float theta = std::atan2(pt.y, pt.x);
      theta = pt.y >= 0 ? theta : theta + M_PI * 2;

      const int r = (phi_rg.max - phi) / phi_rg.delta + 0.5;
      const int c = (theta - theta_rg.min) / theta_rg.delta + 0.5;

      image.at<uint16_t>(r, c) = k;
    }
  }

  ROS_INFO_THROTTLE(
      1, "index image for loop: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  t0 = ros::Time::now();
  tbb::parallel_for(
      tbb::blocked_range<int>(0, cloud.height),
      [&](const tbb::blocked_range<int>& range) {
        for (int i = range.begin(); i < range.end(); ++i) {
          for (int j = 0; j < cloud.width; ++j) {
            int k = i * cloud.width + j;
            const auto& pt = cloud.at(j, i);
            // const auto jj = (j + pt.label) % cloud.width;
            // compute azimuth
            const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (r2 < 0.5 * 0.5) continue;
            const float phi = std::atan2(pt.z, std::hypot(pt.x, pt.y));
            float theta = std::atan2(pt.y, pt.x);
            theta = pt.y >= 0 ? theta : theta + M_PI * 2;

            const int r = (phi_rg.max - phi) / phi_rg.delta + 0.5;
            const int c = (theta - theta_rg.min) / theta_rg.delta + 0.5;

            image.at<uint16_t>(r, c) = k;
          }
        }
      });

  ROS_INFO_THROTTLE(
      1, "index image tbb loop: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  // std_msgs::Header header;
  // pcl_conversions::fromPCL(cloud.header, header);
  // pub.publish(cv_bridge::CvImage(header, "32FC1", image).toImageMsg());
}

// Trying to restore range image
void CloudCb(const sensor_msgs::PointCloud2& cloud_msg) {
  static cv::Mat image;
  ROS_INFO("Got cloud: %u x %u", cloud_msg.height, cloud_msg.width);

  if (image.empty()) {
    image.create(cloud_msg.height, cloud_msg.width, CV_32FC1);
  }

  auto t0 = ros::Time::now();
  sensor_msgs::PointCloud2 copy = cloud_msg;
  ROS_INFO("copy: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  // convert to point cloud is super slow, does not make sense
  t0 = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  ROS_INFO("conversion: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  t0 = ros::Time::now();
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& pt = cloud.at(j, i);
      // image.at<float>(i, j) =
      //     std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      image.at<float>(i, j) = pt.data[3];
    }
  }
  ROS_INFO("to image: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  pub.publish(
      cv_bridge::CvImage(cloud_msg.header, "32FC1", image).toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "os_example");

  ros::NodeHandle pnh("~");
  pub = pnh.advertise<sensor_msgs::Image>("range", 1);
  ros::Subscriber sub = pnh.subscribe("cloud", 5, &CloudCb2);

  ros::spin();

  return 0;
}