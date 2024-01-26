#include "ouster_decoder/driver.h"

int main(int argc , char** argv)
{
    ros::init(argc, argv, "os_driver_node");

    Driver node(ros::NodeHandle("~"));
    ros::spin();

    return 0;
}
