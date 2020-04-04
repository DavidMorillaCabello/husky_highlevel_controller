#include "husky_highlevel_controller/HuskyEmergencyStop.hpp"

namespace husky_highlevel_controller
{
HuskyEmergencyStop::HuskyEmergencyStop(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters_())
    {
        ROS_ERROR("Could not read parameters emergency stop.");
        ros::requestShutdown();
    }

    emergencyStop_ = false;

    subscriberScan_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                            &HuskyEmergencyStop::laserScanCallback_, this);

    serviceClient_ = nodeHandle_.serviceClient<std_srvs::SetBool>(serviceName_);

    ROS_INFO("Successfully launched node.");
}

HuskyEmergencyStop::~HuskyEmergencyStop()
{
}

bool HuskyEmergencyStop::readParameters_()
{
    if (!nodeHandle_.getParam("scan/subscriber_topic", subscriberTopic_))
        return false;
    if (!nodeHandle_.getParam("scan/queue_size", queueSize_))
        return false;
    if (!nodeHandle_.getParam("emergency_stop/emergency_stop_service_name", serviceName_))
        return false;
    if (!nodeHandle_.getParam("emergency_stop/emergency_distance", emergencyDistance_))
        return false;
    return true;
}

void HuskyEmergencyStop::laserScanCallback_(const sensor_msgs::LaserScan &msg)
{

    int minimum = algorithm_.getMinimum(msg.ranges);

    if (!emergencyStop_ && minimum <= emergencyDistance_)
    {
        ROS_INFO("Robot is too closed to an object! Sending stop request");
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
    else if (emergencyStop_ && minimum > emergencyDistance_)
    {
        ROS_INFO("Robot is safe again, sending resume request");
        emergencyStop_ = false;

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
