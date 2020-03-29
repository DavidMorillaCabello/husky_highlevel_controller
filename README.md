# Husky Highlevel Controller

## Overview

This is a package created for the course Programming for Robotics - ROS
which contents are open at the RSL [webpage](https://rsl.ethz.ch/education-students/lectures/ros.html).

It gathers all the exercise sessions following the steps and fulfilling all the requirements.
The style follows the template in [leggedrobotics/ros_best_practices](https://github.com/leggedrobotics/ros_best_practices/tree/master/ros_package_template).

**Keywords:** learn, exercise, ROS Programming

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/davidmorillacabello/husky_highlevel_controller.git
	cd ../
	catkin build

## Usage

Run the complete exercise with:

	roslaunch husky_highlevel_controller husky_highlevel_controller.launch

## Config files

Config file sensors/ 

* **scan.yaml** file to configure the node subscribed to the /scan topic.

## Launch files

* **husky_highlevel_controller.launch:** launch all the nodes for the exercises.
    * husky_gazebo/launch/husky_empty_world.launch
        * 'world_name' world loaded in the Gazebo simulation. Set to: 'worlds/robocup14_spl_field.world'.
        * 'laser_enabled' enables laser in Gazebo. Set to: 'true'.
    * teleop_twist_keyboard.py enables controlling the robot using the keyboard.

## Nodes

### husky_highlevel_controller

Reads the scan measures from the laser scan and computes the minimum distance.

#### Subscribed Topics

* **'/scan'** ([sensor_msgs/LaserScan])
    The scan measures from which the minimum is computed.

#### Parameters

* **'subscriber_topic'** (string, default: "/scan")
    The name of the input topic.

* **'queue_size'** (int, default: 10)
    The size of the queue for the subscriber.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/davidmorillacabello/husky_highlevel_controller/issues).# husky_highlevel_controller
