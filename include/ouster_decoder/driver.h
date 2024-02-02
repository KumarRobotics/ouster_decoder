/*!
 * Kumar Robotics
 * January 2024
 * @breif: a cleaned up version of ousters' old driver. It gets
 * lidar and imu packets from the ouster via the ouster client and 
 * publishes them on a PacketMsg topic.
 * Authors: Jason Hughes
 */

#ifndef DRIVER_H
#define DRIVER_H

#include <fstream>
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "ouster/types.h"
#include "ouster/client.h"
#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"
#include "std_msgs/String.h"

class Driver
{
    public:
        /*!
        * @brief: get and set lidar mode, timestamp mode, ouster metadata
        * and call initRos and initParams 
        * @param: ros nodehandle
        */
        Driver(const ros::NodeHandle& nh);

    private:
    
        /*!
         * @brief: get and set all ROS params
         * as private class varaibles.
        */
        void initParams();
        /*!
        * @brief: initialize lidar packet publisher, imu packet publisher
        * and metadata string publisher.
        */
        void initRos();

        /*!
        * @brief: write the metadata string from the ouster to a 
        * .json file. Default is ...
        * @param: String metadata: a string in json format to be written.
        * @return: bool: true if write was successful, false if it was not.
        */
        bool writeMetadata(const std::string& metadata);
        /*!
        * @brief: callback for the metadata subscription, used for
        * bags containing metadata on a topic rather than in json format.
        * Once metadata is recived pass it to the decoder with the advertiseService.
        * @param: std_msgs::String msg: ros String message containing the metadata.
        */
        void metadataCallback(const std_msgs::String msg);
        /*!
        * @brief: get the lidar and imu packets from the ouster via the
        * ouster client and publish them on their respective topics.
        * @param: sensor_info info: sensor_info struct containing all the metadata.
        */
        int connectionLoop(const ouster::sensor::sensor_info info);
        /*!
        * @brief: convert the metadata to a string and advertise 
        * the metadata on a service for the decoder.
        * @param: sensor_info info: sensor_info struct containing the metadata.
        */
        void advertiseService(const ouster::sensor::sensor_info info);

        // ROS Params
        // @brief: hostname of the ouster, usually the ip address
        std::string hostname_;
        // @brief: where the ouster should send data, an ip address
        // usually the host computer.
        std::string udp_dest_;
        // @brief: which port to get lidar packets on. 
        uint32_t lidar_port_;
        // @brief: which port to get imu packets on.
        uint32_t imu_port_;
        // @brief: set to true if in using a bag.
        bool replay_;
        // @brief: resolution and rate,
        // either 512x10, 512x20, 1024x10, 1024x20, 2048x10
        std::string lidar_mode_arg_;
        // @brief: method used to timestamp measurements,
        // either TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588
        std::string timestamp_mode_arg_;
        // @brief: metadata json file, if in replay mode we will read this file
        // if not we will write metadata to this file.
        std::string meta_file_;

        // ROS
        // @brief: ros nodehandler.
        ros::NodeHandle nh_;
        // @brief: service to send metadata.
        ros::ServiceServer srv_;
        // @brief: lidar packet publisher.
        ros::Publisher lidar_pub_;
        // @brief: imu packet publisher.
        ros::Publisher imu_pub_;
        // @brief: metadata publisher.
        ros::Publisher meta_pub_;
        // @brief: metadata subscriber for bag files.
        ros::Subscriber meta_sub_;

        // Ouster Client
        // @brief: ouster client object seud to talk to the ouster
        // and recieve/send data to it.
        std::shared_ptr<ouster::sensor::client> cli_;

};
#endif
