#include "husky_highlevel_controller/HuskyCollisionStop.hpp"

namespace husky_highlevel_controller
{
HuskyCollisionStop::HuskyCollisionStop(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters_())
    {
        ROS_ERROR("Could not read parameters collision stop.");
        ros::requestShutdown();
    }

    emergencyStop_ = false;

    subscriberScan_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                            &HuskyCollisionStop::imuCallback_, this);

    serviceClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>(serviceName_);

    ROS_INFO("Successfully launched node.");
}

HuskyCollisionStop::~HuskyCollisionStop()
{
}

bool HuskyCollisionStop::readParameters_()
{
    if (!nodeHandle_.getParam("imu/subscriber_topic", subscriberTopic_))
        return false;
    if (!nodeHandle_.getParam("imu/queue_size", queueSize_))
        return false;
    if (!nodeHandle_.getParam("emergency_stop/emergency_stop_service_name", serviceName_))
        return false;
    if (!nodeHandle_.getParam("emergency_stop/collision_threshold", collisionThreshold_))
        return false;
    return true;
}

void HuskyCollisionStop::imuCallback_(const sensor_msgs::Imu &msg)
{
    ROS_INFO("Acceleration: %f", msg.linear_acceleration.x);
    if (!emergencyStop_ && msg.linear_acceleration.x <= collisionThreshold_ && ros::Time::now().toSec() > 5 )
    {
        ROS_INFO("Robot collided to an object! Sending stop request");
        emergencyStop_ = true;

        std_srvs::SetBool service;
        service.request.data = emergencyStop_;
        if(serviceClient_.call(service))
        {
            ROS_INFO("Recieved response: %s",service.response.message.c_str());
        } else {
            ROS_ERROR("Failed to call service add_two_ints");
        }
    }
}

} // namespace husky_highlevel_controller
