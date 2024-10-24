/*!
* Kumar Robotics
* January 2024 Refactor
* @brief: starter for ouster driver ros node
* Authors: Jason Hughes
*/

#include "ouster_decoder/shared_driver.h"
#include "nodelet/loader.h"

int main(int argc , char** argv)
{
    ros::init(argc, argv, "os_driver_nodelet");

    nodelet::Loader nodelet;

    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();

    try
    {
        nodelet.load(nodelet_name, "OusterDriver/Driver", remap, nargv);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
    }

    return 0;
}
