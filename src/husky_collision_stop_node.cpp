#include <ros/ros.h>
#include <husky_highlevel_controller/HuskyCollisionStop.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_collision_stop");
    ros::NodeHandle nodeHandle("~");

    husky_highlevel_controller::HuskyCollisionStop huskyCollisionStop(nodeHandle);

    ros::spin();
    return 0;
}