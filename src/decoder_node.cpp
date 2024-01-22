/* January 2024
*  As part of the refactor for upgrades to ouster_ros v0.10+
*  Node start script for ouster decoder
*/

#include "ouster_decoder/decoder.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "os_decoder_node");

    Decoder node(ros::NodeHandle("~"));
    ros::spin();

    return 0;
}

