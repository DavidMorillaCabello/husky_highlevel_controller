#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller
{
HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
    if (!readParameters_())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    subscriberScan_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_,
                                            &HuskyHighlevelController::topicsCallback_, this);

    //serviceServer_ = nodeHandle.advertiseService(...) NOT USED

    publisherVel_ = nodeHandle_.advertise<geometry_msgs::Twist>(publisherTopic_, queueSize_);
    pillarVisPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    ROS_INFO("Successfully launched node.");
}

HuskyHighlevelController::~HuskyHighlevelController()
{
}

bool HuskyHighlevelController::readParameters_()
{
    if (!nodeHandle_.getParam("scan/subscriber_topic", subscriberTopic_))
        return false;
    if (!nodeHandle_.getParam("scan/queue_size", queueSize_))
        return false;
    if (!nodeHandle_.getParam("vel_control/P_linear", PLinear_))
        return false;
    if (!nodeHandle_.getParam("vel_control/P_angular", PAngular_))
        return false;
    if (!nodeHandle_.getParam("vel_control/publisher_name", publisherTopic_))
        return false;
    return true;
}

void HuskyHighlevelController::topicsCallback_(const sensor_msgs::LaserScan &msg)
{

    int index = algorithm_.getMinimumIndex(msg.ranges);

    float angle = msg.angle_min + msg.angle_increment * index;

    algorithm_.p2c(msg.ranges[index], angle, xPillar_, yPillar_);

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = xPillar_ * PLinear_;

    cmd_vel.angular.z = -yPillar_ * PAngular_;

    transformMarker_(xPillar_, yPillar_, pillarOdomTf_);

    publishMarker_(pillarOdomTf_);
    publisherVel_.publish(cmd_vel);
}

void HuskyHighlevelController::transformMarker_(const float xSensor, const float ySensor, tf::Transform &markerTf)
{
    tf::StampedTransform transform;
    tf::Transform sensorTransform;

    sensorTransform.getOrigin().setX(xSensor);
    sensorTransform.getOrigin().setY(ySensor);
    
    try
    {
        tfListener_.lookupTransform("/odom", "/base_laser", ros::Time(0), transform);
        ROS_INFO("transform X: %f\ntransform Y: %f\nsensor X: %f\nsensor Y: %f\n",transform.getOrigin().x(),transform.getOrigin().y(),xSensor,ySensor);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    
    markerTf = transform * sensorTransform;

}

void HuskyHighlevelController::publishMarker_(const tf::Transform &markerTF)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "husky_highlevel_controller";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = markerTF.getOrigin().x();
    marker.pose.position.y = markerTF.getOrigin().y();
    marker.pose.position.z = markerTF.getOrigin().z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    pillarVisPub_.publish(marker);
}

} // namespace husky_highlevel_controller
