#!/bin/bash
# record_ball.sh

source "./countdown.sh"

echo "Prepare to start recording rosbag"
countdown 10

# start recording ball topic as background process
rosbag record /ball_pos -O ~/catkin_ws/src/aftcode2_0/rosbags/ball_data_$(date +%s) &

# Capture PID of rosbag record
CMD_PID=$!
echo "Started recording rosbag with PID: $CMD_PID"

# record 5 seconds of data
countdown 5

# Send the SIGINT signal to the rosbag process
kill -INT "$CMD_PID"

# Close shell
echo "SIGINT sent to PID: $CMD_PID"
echo "Closing shell."
