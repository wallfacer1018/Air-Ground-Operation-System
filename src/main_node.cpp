#include <ros/ros.h>
#include "fsm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh("~");

    FSM fsm;
    fsm.init(nh);

    ros::spin();

    return 0;
}
