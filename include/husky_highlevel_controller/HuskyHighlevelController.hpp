#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>

// STD
#include <string>

namespace husky_highlevel_controller
{

     /*!
     *  Main class for the node to handle the ROS interfacing.
     */
class HuskyHighlevelController
{
public:
     /*
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
     HuskyHighlevelController(ros::NodeHandle &nodeHandle);

     /*
     * Destructor
     */
     virtual ~HuskyHighlevelController();

private:
     /*!
     * Reads and verifies the ROS parameters.
     * @return true if succesful.
     */
     bool readParameters_();

     /*!
     * ROS topic callback method.
     * @param message the received message.
     */
     void laserScanCallback_(const sensor_msgs::LaserScan &msg);

     /*!
     * ROS method to transform the markers position.
     * @param x the x position of the pillar.
     * @param y the y position of the pillar.
     */
     void transformMarker_(const float xSensor, const float ySensor, tf::Transform &markerTf);

     /*!
     * ROS method to publish markers knowing the position.
     * @param x the x position of the pillar.
     * @param y the y position of the pillar.
     */
     void publishMarker_(const tf::Transform &markerTf);

     /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if succesful, false otherwise.
     */
     bool serverCallback_(std_srvs::SetBoolRequest &request, std_srvs::SetBoolResponse &response);

     //! ROS node handle.
     ros::NodeHandle &nodeHandle_;

     //! ROS topic subscriber.
     ros::Subscriber subscriberScan_;

     //! ROS topic name to subscribe to recieve LaserScan info.
     std::string subscriberTopic_;

     //! ROS subscriber queue size.
     int queueSize_;

     //! ROS service server.
     ros::ServiceServer serviceServer_;

     //! ROS service name to create the server for.
     std::string serviceName_;

     //! ROS publisher to the cmd_vel topic
     ros::Publisher publisherVel_;

     //! ROS topic name to public cmd_vel to.
     std::string publisherTopic_;

     //! Gain for the P controller.
     float PLinear_, PAngular_;

     //! ROS publisher for the visualization of the pillar.
     ros::Publisher pillarVisPub_;

     //! Algorithm computation object.
     Algorithm algorithm_;

     //! Pillar position with respect the base_laser tf
     float xPillar_, yPillar_;

     //! ROS listener for the tf transformations
     tf::TransformListener tfListener_;

     //! Pillar position with respect the odom tf
     tf::Transform pillarOdomTf_;

     //! Variable to trigger the emergency stop
     bool emergencyStop_;
};
} // namespace husky_highlevel_controller