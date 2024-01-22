/* January 2024
*  header file for vizualization class
*/

#ifndef VIZ_H
#define VIZ_H

#include <cv_bridge/cv_bridge.h>
#include <fmt/core.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Viz 
{
    public:
        explicit Viz(const ros::NodeHandle& pnh);

        void cameraCb(const sensor_msgs::ImageConstPtr& image_ptr,
                      const sensor_msgs::CameraInfoConstPtr& cinfo_ptr);

    private:
        cv::Mat applyCmap(const cv::Mat& input,
                          int cmap,
                          uint8_t bad_color);

        ros::NodeHandle pnh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber sub_camera_;
        std::string cmap_{"gray"};
};
#endif
