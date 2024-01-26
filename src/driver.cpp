#include "ouster_decoder/driver.h"

Driver::Driver(const ros::NodeHandle& nh) : nh_(nh)
{
    initParams();
    initRos();

    ouster::sensor::sensor_info info;
    std::string metadata;

    // set lidar mode
    ouster::sensor::lidar_mode lidar_mode = ouster::sensor::MODE_UNSPEC;
    if (lidar_mode_arg_.size())
    {
        if (replay_) ROS_WARN("Lidar mode set in replay mode, will be ignored");

        lidar_mode = ouster::sensor::lidar_mode_of_string(lidar_mode_arg_);
        if (!lidar_mode)
        {
            ROS_ERROR("Invalid lidar mode %s", lidar_mode_arg_.c_str());
            ros::shutdown();
        }
    }

    ouster::sensor::timestamp_mode timestamp_mode = ouster::sensor::TIME_FROM_UNSPEC;
    if (timestamp_mode_arg_.size())
    {
        if (replay_) ROS_WARN("Timestamp mode set in replay mode, will be ignored");

        timestamp_mode = ouster::sensor::timestamp_mode_of_string(timestamp_mode_arg_);
        if(!timestamp_mode)
        {
            ROS_ERROR("Invalid timestamp mode %s, exiting", timestamp_mode_arg_.c_str());
            ros::shutdown();
        }
    }

    if (!replay_ && (!hostname_.size() || !udp_dest_.size()))
    {
        ROS_ERROR("Must specify both hostname and udp destination when not in replay mode");
        ros::shutdown();
    }

    if (replay_)
    {
        try
        {
            ROS_INFO("Running in replay mode");
            info = ouster::sensor::metadata_from_json(meta_file_); 
            advertiseService(info);

        } catch (const std::runtime_error& e)
        {
            ROS_ERROR("Error running in replay mode: %s", e.what());
        }
    }
    else
    {
        try
        {
            ROS_INFO("Connecting to sensor with hostname: %s", hostname_.c_str());
            ROS_INFO("Sending data to udp destination: %s", udp_dest_.c_str());

            cli_ = ouster::sensor::init_client(hostname_,
                                               udp_dest_,
                                               lidar_mode,
                                               timestamp_mode,
                                               lidar_port_,
                                               imu_port_);
            if (!cli_)
            {
                ROS_ERROR("Failed to initialize sensor at %s", hostname_.c_str());
                ros::shutdown();
            }
            metadata = ouster::sensor::get_metadata (*cli_, 20, true);
            if (meta_file_.empty())
            {
                meta_file_ = hostname_+".json";
                ROS_INFO("No metadata json specified, using %s", meta_file_.c_str());
            }
            if (!writeMetadata(metadata))
            {
                ROS_ERROR("Couldn't write metadata to json, continueing");
            }
            info = ouster::sensor::parse_metadata(metadata);

            advertiseService(info);
        } catch(const std::exception& e)
        {
            ROS_ERROR("Error connection to sensor: %s", e.what());
        }
    }
    ROS_INFO("Using lidar_mode: %s", ouster::sensor::to_string(info.mode).c_str());
    ROS_INFO("%s sn: %s firmware rev: %s",
             info.prod_line.c_str(),
             info.sn.c_str(),
             info.fw_rev.c_str());
    if (!replay_) int success = connectionLoop(info);
}

bool Driver::writeMetadata(const std::string& metadata)
{
    std::ofstream ofs;
    ofs.open(meta_file_);
    ofs << metadata << std::endl;
    ofs.close();
    if (ofs) 
    {
        ROS_INFO("Wrote metadata to %s", meta_file_.c_str());
    } 
    else 
    {
        ROS_WARN(
            "Failed to write metadata to %s; check that the path is valid. If "
            "you provided a relative path, please note that the working "
            "directory of all ROS nodes is set by default to $ROS_HOME",
            meta_file_.c_str());
        return false;
    }
    return true;
}

int Driver::connectionLoop(const ouster::sensor::sensor_info info)
{
    ouster::sensor::packet_format pf = ouster::sensor::get_format(info);

    ouster_ros::PacketMsg lidar_packet, imu_packet;
    lidar_packet.buf.resize(pf.lidar_packet_size + 1);
    imu_packet.buf.resize(pf.imu_packet_size + 1);
    
    ROS_INFO("Publishing Packets...");
    while (ros::ok())
    {
        ouster::sensor::client_state state = ouster::sensor::poll_client(*cli_);
        if (state == ouster::sensor::EXIT)
        {
            ROS_INFO("Client: caught signal, exiting");
            ros::shutdown();
        }
        if (state & ouster::sensor::CLIENT_ERROR)
        {
            ROS_ERROR("Client: returned error");
            ros::shutdown();
        }
        if (state & ouster::sensor::LIDAR_DATA)
        {
            if (ouster::sensor::read_lidar_packet(*cli_, lidar_packet.buf.data(), pf))
            {
                lidar_pub_.publish(lidar_packet);
            }
        }
        if (state & ouster::sensor::IMU_DATA)
        {
            if (ouster::sensor::read_imu_packet(*cli_, imu_packet.buf.data(), pf))
            {
                imu_pub_.publish(imu_packet);
            }
        }
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}

void Driver::advertiseService(const ouster::sensor::sensor_info info)
{
    std::string metadata = info.original_string();//ouster::sensor::to_string(info);

    if (srv_)
    {
        srv_.shutdown();
    }
    srv_ = nh_.advertiseService<ouster_ros::GetMetadata::Request, ouster_ros::GetMetadata::Response>(
         "get_metadata",
         [metadata](ouster_ros::GetMetadata::Request&, ouster_ros::GetMetadata::Response& res) {
           if (metadata.empty()) return false;
           res.metadata = metadata;
           return true;
         });
    ROS_INFO("Advertising Metadata on service %s.", srv_.getService().c_str());
}

void Driver::initRos()
{
    lidar_pub_ = nh_.advertise<ouster_ros::PacketMsg>("lidar_packets", 1280);
    imu_pub_ = nh_.advertise<ouster_ros::PacketMsg>("imu_packets", 100);
}

void Driver::initParams()
{
    hostname_ = nh_.param("sensor_hostname", std::string{});
    udp_dest_ = nh_.param("udp_dest", std::string{});
    lidar_port_  = nh_.param("lidar_port", 0);
    imu_port_    = nh_.param("imu_port", 0);
    replay_          = nh_.param("replay", false);
    lidar_mode_arg_ = nh_.param("lidar_mode", std::string{});
    timestamp_mode_arg_ = nh_.param("timestamp_mode", std::string{});
    nh_.param<std::string>("udp_profile_lidar", udp_profile_lidar_arg_, "");
    nh_.param<std::string>("metadata", meta_file_, "");
}
