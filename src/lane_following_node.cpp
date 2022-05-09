#include "turtlebot3_lane_following/robot_pid.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");

  RobotPID robot_pid;

  ros::spin();

  return 0;
}
