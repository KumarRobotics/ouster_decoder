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
        Driver(const ros::NodeHandle& nh);

    private:
    
        void initParams();
        void initRos();

        bool writeMetadata(const std::string& metadata);
        int connectionLoop(const ouster::sensor::sensor_info info);
        void advertiseService(const ouster::sensor::sensor_info info);

        std::string hostname_;
        std::string udp_dest_;
        uint32_t lidar_port_;
        uint32_t imu_port_;
        bool replay_;
        std::string lidar_mode_arg_;
        std::string timestamp_mode_arg_;
        std::string udp_profile_lidar_arg_;
        std::string meta_file_;

        ros::NodeHandle nh_;
        ros::ServiceServer srv_;
        ros::Publisher lidar_pub_;
        ros::Publisher imu_pub_;
        ros::Publisher meta_pub_;
        ros::Subscriber meta_sub_;

        std::shared_ptr<ouster::sensor::client> cli_;

};
#endif
