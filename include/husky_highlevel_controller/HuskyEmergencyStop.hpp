#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

// STD
#include <string>

namespace husky_highlevel_controller
{

/*!
 *  Main class for the node to handle the ROS interfacing.
 */
class HuskyEmergencyStop
{
public:
    /*
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    HuskyEmergencyStop(ros::NodeHandle &nodeHandle);

    /*
     * Destructor
     */
    virtual ~HuskyEmergencyStop();

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

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriberScan_;

    //! ROS topic name to subscribe to recieve LaserScan info.
    std::string subscriberTopic_;

    //! ROS subscriber queue size.
    int queueSize_;

    //! ROS service server.
    ros::ServiceClient serviceClient_;

    //! ROS service name to call the server.
    std::string serviceName_;

    //! Configurable parameter to store the distance to trigger the emergency stop.
    float emergencyDistance_;

    //! Variable to hold if the emergency stop was triggered.
    bool emergencyStop_;

    //! Algorithm computation object.
    Algorithm algorithm_;
};
} // namespace husky_highlevel_controller