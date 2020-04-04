# Husky Highlevel Controller

## Overview

!!!IMPORTANT NOTICE: If having this repo as public could be incorrect or detrimental in any way, please notify me asap!!!

This is a package created for the course Programming for Robotics - ROS
which contents are open at the RSL [webpage](https://rsl.ethz.ch/education-students/lectures/ros.html).
I completed all the exercises from the course during my free time in a self-study way during a week as a way
of practicing and improving my ROS programming.

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

Run the different exercises with:

* Follow the column and display markers in rviz (with tf listener).

	roslaunch husky_highlevel_controller husky_highlevel_controller.launch

* Displays the info from the rosbag provided by the RSL webpage and shows it in Rviz with the point cloud.

	roslaunch husky_highlevel_controller load_husky_bag.launch

* Follow the column until the distance is less than a certain limit. If the column gets far again, the following is reactivated.

    roslaunch husky_highlevel_controller husky_emergency_stop.launch

* Follow the column until the IMU detects a collision.

    roslaunch husky_highlevel_controller husky_collision_stop.launch

## Config files

robot/

* **control.yaml** file to configure the parameters for the control of the robot.

* **emergency_stop.yaml** file to configure the parameters for emergency stop by distance of the robot.

* **collision_stop.yaml** file to configure the parameters for emergency stop by collision of the robot.

sensors/ 

* **scan.yaml** file to configure the nodes subscribed to the /scan topic.

* **imu.yaml** file to configure the nodes subscribed to the /imu/data topic.

## Launch files

* **husky_highlevel_controller.launch:** launch all the nodes for the exercises.
    * husky_gazebo/launch/husky_empty_world.launch
        * 'world_name' world loaded in the Gazebo simulation. Set to: 'worlds/robocup14_spl_field.world'.
        * 'laser_enabled' enables laser in Gazebo. Set to: 'true'.
    * teleop_twist_keyboard.py enables controlling the robot using the keyboard.

## Nodes

### husky_highlevel_controller

Reads the scan measures from the laser scan and commads the robot to go for the column.
It also publishes a marker for the column position for its visualization in Rviz using
a TF listener and a manual transformation from the laser scan to the odom frames.
It finally implements a service to stop the robot in case of emergency.

#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])
    The scan measures from which the minimum is computed.

#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])
    The velocity command for the robot.

* **'/visualization_marker'** ([visualization_msgs/Marker])
    Visualization marker for the position of the column.

#### Services

* **`emergency_stop`** ([std_srvs/SetBool])
    Set a stopped state for the robot when the request data is true and allows movement again whhen request data is false.
    It can be triggered from the consol with:
        rosservice call /emergency_stop "data: true"
    It's used to stop the robot from the emergency stop and collision stop nodes.

#### Parameters

* **`subscriber_topic`** (string, default: "/scan")
    The name of the laser scan topic.

* **`queue_size`** (int, default: 10)
    The size of the queue for the subscriber and publisher.

* **`p_linear`** (float, default: 1)
    The proporcional value for the linear velocity control.

* **`p_angular`** (float, default: 0.1)
    The proporcional value for the angular velocity control.

* **`subscriber_topic`** (string, default: "/cmd_vel")
    The name of the velocity command topic.

* **`service_name`** (string, default: "/emergency_stop")
    The name of the emergency stop service.

### husky_emergency_stop

Reads the scan measures from the laser scan and computes the minimum.
Calls the emergency_stop service in case that the minimum is less than a configurable measure.
If the robot is stopped and the distance is high again, it starts the movement again.

#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan])
    The scan measures from which the minimum is computed.

#### Services

* **`emergency_stop`** ([std_srvs/SetBool])
    Sends a stop request if the robot is too close to the column.
    Sends a resume request if the robot is far from the column again.

#### Parameters

* **`subscriber_topic`** (string, default: "/scan")
    The name of the laser scan topic.

* **`queue_size`** (int, default: 10)
    The size of the queue for the subscriber and publisher.

* **`service_name`** (string, default: "/emergency_stop")
    The name of the emergency stop service.

* **`emergency_distance`** (float, default: 1)
    The limit distance to trigger the emergency_stop service 
    in case that the minimum laser scan detection is below it.

### husky_collision_stop

Reads the measures from the imu and extracts the x acceleration.
If there is a big acceleration after the robot has started in a negative x direction,
it means that something has collided with the frontal part of the robot. Therefore,
the emergency_stop service is called to stop the robot.

#### Subscribed Topics

* **`/imu/data`** ([sensor_msgs/Imu])
    The imu measures from which the x acceleration is extracted.

#### Services

* **`emergency_stop`** ([std_srvs/SetBool])
    Sends a stop request if the robot is has collided with anything 
    (x acceleration value bigger than a configurable measure in its negative direction).

#### Parameters

* **`subscriber_topic`** (string, default: "/scan")
    The name of the laser scan topic.

* **`queue_size`** (int, default: 10)
    The size of the queue for the subscriber and publisher.

* **`service_name`** (string, default: "/emergency_stop")
    The name of the emergency stop service.

* **`collision_threshold`** (float, default: -1)
    The maximum value of the x acceleration to trigger the emergency_stop service.
    Notice that it's a negative value to detect frontal collisions.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/davidmorillacabello/husky_highlevel_controller/issues).# husky_highlevel_controller
