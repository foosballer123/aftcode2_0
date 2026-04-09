#!/bin/bash

# Source ROS (IMPORTANT for roslaunch + remote machines)
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Argument (angular x)
ANGULAR_X=$1

# Optional defaults
RATE=10
DURATION=0   # 0 = run forever

# Debug print
echo "Publishing angular.x = $ANGULAR_X"

# Publish
if [ "$DURATION" -gt 0 ]; then
    timeout $DURATION rostopic pub /omega_d geometry_msgs/Twist \
    "{angular: {x: $ANGULAR_X, y: 0.0, z: 0.0}}" -r $RATE
else
    rostopic pub /omega_d geometry_msgs/Twist \
    "{angular: {x: $ANGULAR_X, y: 0.0, z: 0.0}}" -r $RATE
fi
