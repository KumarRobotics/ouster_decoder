/*!
 * Kumar Robotics
 * Januart 2024 refactor
 * @breif: decodes incoming lidar and imu packets and publishes them 
 * on appropriate ROS topics
 * Authors: Chao Qu and Jason Hughes
 */

#ifndef DECODER_H
#define DECODER_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"

#include "lidar.h"

constexpr double kDefaultGravity = 9.807;

class Decoder 
{
    public:
        /*!
        * @breif: call to set ros params, initial ros and ouster
        * @param: ros nodehandle
        */
        explicit Decoder(const ros::NodeHandle& pnh);

        // No copy no move
        Decoder(const Decoder&) = delete;
        Decoder& operator=(const Decoder&) = delete;
        Decoder(Decoder&&) = delete;
        Decoder& operator=(Decoder&&) = delete;

        /*!
        * @breif: lidar packet callback
        * @param: PacketMsg, ros msg containing lidar packet
        * data.
        */
        void LidarPacketCb(const ouster_ros::PacketMsg& lidar_msg);
        /*!
        * @brief: imu packet callback
        * @param: PacketMsg, ros msg containing imu packet
        */
        void ImuPacketCb(const ouster_ros::PacketMsg& imu_msg);

    private:
        /*!
        * @brief: initialize ros publishers/ subscribers and frames
        */
        void InitRos();
        /*!
        * @breif: initialize all ros params.
        */
        void InitParams();
        /*!
        * @breif: get metadata from ros client from driver, 
        * call lidar initializers
        */
        void InitOuster();
        /*!
        * @breif: initialize LidarModel object, 
        * generate camera info for topic
        * @param: string of metadata recieved from ros client.
        */
        void InitModel(const std::string& metadata);
        /*!
        * @breif: Allocate the memory for pointcloud 
        * based on the number of subscans gotten by the "divide" param.
        * @param: lidar model object, containing all the lidar information.
        */
        void InitScan(const LidarModel& model);
        /*!
        * @breif: send imu and lidar transforms to tf.
        * @param: lidar model object, containing all the lidar information.
        */
        void SendTransform(const LidarModel& model);

        /*!
        * @breif: check if decoder is still waiting for alignment to mid 0.
        * @param: mid, column measurment id from the column buffer.
        * @return: true if mid == 0
        */
        [[nodiscard]] bool NeedAlign(int mid);

        /*!
        * @breif: publish the pointcloud, range and signal images, and 
        * camera info. Reset the lidar scan.
        */
        void PublishAndReset();

        /*!
        * @breif: record processing time of lidar callback, print warning if it 
        * exceeds window between two packets
        * @param: ros time when the lidar packet was recieved.
        */
        void Timing(const ros::Time& start) const;

        // ROS
        // @brief: ros noehandler.
        ros::NodeHandle pnh_;
        // @brief: tos image transporter.
        image_transport::ImageTransport it_;
        // @breif: lidar imu and metadata subscribers.
        ros::Subscriber lidar_sub_, imu_sub_, meta_sub_;
        // @breif: point cloud and imu publisher.
        ros::Publisher cloud_pub_, imu_pub_;
        // @breif: range and signal image publishers.
        ros::Publisher range_pub_, signal_pub_;
        // @breif: camera info publisher.
        image_transport::CameraPublisher camera_pub_;
        // @brief: tf2 static transform broadcaster
        tf2_ros::StaticTransformBroadcaster static_tf_;
        // @brief: frames, defined in launch file and gotten
        // as ros params.
        std::string sensor_frame_, lidar_frame_, imu_frame_;

        // DATA
        // @breif: object to hold incoming lidar data
        LidarScan scan_;
        // @breif: object to hold lidar metadata
        LidarModel model_;
        // @brief: ros msg for camera info
        sensor_msgs::CameraInfoPtr cinfo_msg_;

        // PARAMS
        // @breif: gravity
        double gravity_{};
        // @brief: replay mode will reinitialize on jump
        bool replay_{false};            
        // @breif: whether to align scan
        bool need_align_{true};         
        // @breif: discrete time accelerometer noise variance
        double acc_noise_var_{};        
        // @breif: discrete time gyro noise varaince
        double gyr_noise_var_{};        
        // @breif: scal signal visualization
        double vis_signal_scale_{1.0};  
}; 
#endif
