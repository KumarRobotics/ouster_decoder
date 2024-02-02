/*!
 * Kumar Robotics
 * January 2024 Refactor
 * @brief: starter for the ros decoder node
 * Authors: Jason Hughes
 */  

#include "ouster_decoder/decoder.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "os_decoder_node");

    Decoder node(ros::NodeHandle("~"));
    ros::spin();

    return 0;
}

