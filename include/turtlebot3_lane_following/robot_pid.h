#ifndef ROBOT_PID_H
#define ROBOT_PID_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <turtlebot3_lane_detection/line_msg.h>

#include <iostream>

class RobotPID
{
private:
    ros::NodeHandle nh;
    ros::Subscriber lane_detection_subscriber;
    ros::Subscriber color_sensor_detection_subscriber;
    ros::Publisher command_publisher;

    /**  Parameters from roslaunch **/
    double robot_linear_speed;
    double robot_angular_speed;
    double propotionnal_constant;
    double integration_constant;
    double derivation_constant;
    double angle_order;

    std::string sub_topic_name;

    double cumulated_error;
    double previous_error;

    geometry_msgs::Twist robot_command;

    void laneDetectionCallback(const turtlebot3_lane_detection::line_msg &msg);
    void ColorSensorDetectionCallback(const std_msgs::Float32::ConstPtr &msg);

    double bang_bang(double measure_angle);
    double P(double measure_angle);
    double PI(double measure_angle);
    double PID(double measure_angle);

public:
    RobotPID();
    ~RobotPID(){};

    void stopRobot();
};

#endif