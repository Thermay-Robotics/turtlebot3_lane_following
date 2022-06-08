#include "turtlebot3_lane_following/robot_pid.h"

RobotPID::RobotPID() : cumulated_error(0), previous_error(0)
{
    nh.getParam("/lane_following_node/angle_order", angle_order);
    nh.getParam("/lane_following_node/robot_linear_speed", robot_linear_speed);
    nh.getParam("/lane_following_node/robot_angular_speed", robot_angular_speed);
    nh.getParam("/lane_following_node/propotionnal_constant", propotionnal_constant);
    nh.getParam("/lane_following_node/integration_constant", integration_constant);
    nh.getParam("/lane_following_node/derivation_constant", derivation_constant);
    nh.getParam("/lane_following_node/sensor_topic", sub_topic_name);

    lane_detection_subscriber = nh.subscribe(sub_topic_name, 1, &RobotPID::laneDetectionCallback, this);
    color_sensor_detection_subscriber = nh.subscribe(sub_topic_name, 1, &RobotPID::ColorSensorDetectionCallback, this);

    command_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    robot_command.linear.x = robot_linear_speed;
    robot_command.linear.y = 0;
    robot_command.linear.z = 0;
    robot_command.angular.x = 0; 
    robot_command.angular.y = 0; 
    robot_command.angular.z = 0; 
}

void RobotPID::ColorSensorDetectionCallback(const std_msgs::Float32::ConstPtr &msg){
    robot_command.angular.z = P(msg->data);

    command_publisher.publish(robot_command);

    std::cout << "angular command : " << robot_command.angular.z << std::endl;
}

void RobotPID::laneDetectionCallback(const turtlebot3_lane_detection::line_msg &msg){
    robot_command.angular.z = P(msg.angle);

    command_publisher.publish(robot_command);

    std::cout << "angular command : " << robot_command.angular.z << std::endl;
}

double RobotPID::bang_bang(double measure_angle){
    double error = measure_angle - angle_order;
    double calculated_order;
    if(measure_angle - angle_order < 0)
        return -robot_angular_speed;
 
    return robot_angular_speed;
}

double RobotPID::P(double measure_angle){
    double error = measure_angle - angle_order;
    double calculated_order;
    
    return propotionnal_constant * error;

}

double RobotPID::PI(double measure_angle){
    double error = measure_angle - angle_order;

    cumulated_error += error;

    return P(measure_angle) + integration_constant * cumulated_error;
}

double RobotPID::PID(double measure_angle){
    double error = measure_angle - angle_order;

    double correction_order = PI(measure_angle) + derivation_constant*(error - previous_error);

    previous_error = error;

    return correction_order;
}