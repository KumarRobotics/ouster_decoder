#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/mat.hpp>

ros::Publisher pub;
using PointT = pcl::PointXYZRGBL;
using CloudT = pcl::PointCloud<PointT>;

void CloudCb2(const CloudT& cloud) {
  static cv::Mat image;
  static cv::Mat image2;

  if (image.empty()) {
    image.create(cloud.height, cloud.width, CV_32FC1);
    image2.create(cloud.height, cloud.width, CV_32FC3);
  }

  auto t0 = ros::Time::now();
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& pt = cloud.at(j, i);
      // const auto jj = (j + pt.label) % cloud.width;

      image.at<float>(i, j) =
          std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      // image.at<float>(i, j) = pt.curvature;
    }
  }
  ROS_INFO("to range image: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  t0 = ros::Time::now();
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      const auto& pt = cloud.at(j, i);
      auto* px = image.ptr<cv::Vec3f>(i, j);
      px[0] = pt.x;
      px[1] = pt.y;
      px[2] = pt.z;
    }
  }
  ROS_INFO("to xyz image: %f ms", (ros::Time::now() - t0).toSec() * 1e3);

  std_msgs::Header header;
  pcl_conversions::fromPCL(cloud.header, header);
  pub.publish(cv_bridge::CvImage(header, "32FC1", image).toImageMsg());
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