#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
    HuskyHighlevelController(ros::NodeHandle& nodeHandle);
    
    /*
     * Destructor
     */
    virtual ~HuskyHighlevelController();

    private:
        /*!
         * Reads and verifies the ROS parameters.
         * @return true if succesful.
         */
        bool readParameters();

        /*!
         * ROS topic callback method.
         * @param message the received message.
         */
        void topicsCallback(const sensor_msgs::LaserScan& msg);

        /*!
         * ROS service server callback.
         * @param request the request of the service.
         * @param response the provided response.
         * @return true if succesful, false otherwise.
         */
        //bool serviceCallback(...) NOT USED

        //! ROS node handle.
        ros::NodeHandle& nodeHandle_;

        //! ROS topic subscriber.
        ros::Subscriber subscriber_;

        //! ROS topic name to subscribe to.
        std::string subscriberTopic_;

        //! ROS subscriber queue size.
        int queueSize_;
        //! ROS service server.
        //ros::ServiceServer serviceServer_; NOT USED

        //! Algorithm computation object.
        Algorithm algorithm_;
    };
} // namespace husky_highlevel_controller