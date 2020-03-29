#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller
{
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                        &HuskyHighlevelController::topicsCallback, this);

    //serviceServer_ = nodeHandle.advertiseService(...) NOT USED

    ROS_INFO("Successfully launched node.");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

bool HuskyHighlevelController::readParameters(){
    if(!nodeHandle_.getParam("scan/subscriber_topic", subscriberTopic_)) return false;
    if(!nodeHandle_.getParam("scan/queue_size", queueSize_)) return false;
    return true;
}

void HuskyHighlevelController::topicsCallback(const sensor_msgs::LaserScan& msg){
    ROS_INFO("Min distance: %f\n", algorithm_.getMinimum(msg.ranges));
}
} // namespace husky_highlevel_controller
