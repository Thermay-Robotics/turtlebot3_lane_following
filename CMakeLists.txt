cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_lane_following)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  turtlebot3_lane_detection
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs turtlebot3_lane_detection
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(lane_following_node src/lane_following_node.cpp src/robot_pid.cpp )


## Specify libraries to link a library or executable target against
target_link_libraries(lane_following_node
  ${catkin_LIBRARIES}
)

add_dependencies(lane_following_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

