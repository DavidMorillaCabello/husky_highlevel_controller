#include <ros/ros.h>
#include <husky_highlevel_controller/HuskyEmergencyStop.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_emergency_stop");
    ros::NodeHandle nodeHandle("~");

    husky_highlevel_controller::HuskyEmergencyStop huskyEmergencyStop(nodeHandle);

    ros::spin();
    return 0;
}