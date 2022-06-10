# turtlebot3_lane_following

PID Turtlebot3 lane following projet

# Notice

* The node subscribes to angle topic of the lane detection node. This value is used to compute the robot command in order to follow the lane. 

### Subscribed Topics

* ```/lane_detection/angle``` ([turtlebot3_lane_detection/line_msg])
    Angle of the lane to the robot

### Published Topics

* ``` /cmd_vel``` ([geometry_msgs/Twist])
    Publishes the computed linear and angular speed of the robot

# How to build
```
cd ~/catkin_ws/src/
git clone https://github.com/Thermay-Robotics/turtlebot3_lane_following.git
cd ~/catkin_ws
catkin_make
```
# Run

Launch the camera node

Launch detection node

``` roslaunch turtlebot3_lane_detection turtlebot3_lane_detection.launch ```

if you want to run it with the camera, you can launch

``` roslaunch turtlebot3_lane_detection turtlebot3_detection_realsense.launch```

Launch PID node

``` roslaunch turtlebot3_lane_following lane_following_node.launch```

## Additional parameters

* order ([double], default = 0)

    Command of the PID

* linear_speed ([double]), default=0.5)

    Max linear speed of the robot

* angular_speed ([double], default=0.1) 

    Max angular speed of the robot

# Test environment

``` 
Ubuntu 20.04 LTS
ROS Noetic
```

``` 
Jetson Nano
Ubuntu 18.04 
ROS Melodic
```

