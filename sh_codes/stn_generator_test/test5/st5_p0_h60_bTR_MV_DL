#!/bin/bash

# 1. Get the directory of the current script
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# 2. Source the file from the parent directory
source "$SCRIPT_DIR/../get_params.sh"
source "$SCRIPT_DIR/../countdown.sh"

# 3. Get parameters based on the file name
get_params

# Check encoder value; if not 1, publish \omega_d until 1.
echo "Waiting for motor 1 to initialize..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/init_agent.py
echo "Initialized."

# Move player to a desired initial position.
echo "Waiting for motor 1 to move..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/mov_agent.py --desired_pos=$y1
echo "Moved."

# Start MPC Solver node with desired p-horizon (using argparse)
echo "Starting Y-Solver with --prediction_steps=$h..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/MPC_Y_Solver.py --prediction_steps=$h & # --> The ampersand '&' ensures that the python scripts runs as a background process so that the next line can execute

MPC_PID=$!
echo "Started MPC Solver with PID: $MPC_PID"

# Wait...
sleep 7

# Start recording rosbag with topics \omega_d \ball_pos \motor1_pos \motor2_rad
echo "Recording topics \omega_d \ball_pos \motor1_pos \motor2_rad ..."
rosbag record /camera/color/image_raw /omega_d /ball_pos /motor1_pos -O ~/catkin_ws/src/aftcode2_0/rosbags/"$base_name"_$(date +"%y_%m_%d_%H_%M_%S") &

# Capture PID of rosbag record
BAG_PID=$!
echo "Started recording rosbag with PID: $BAG_PID"

# Start ball simulation node and print 'mode' and 'number'
echo "Starting 'ball_sim_tests.py' with 'mode=1' and 'test=$test'..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/ball_sim_tests.py --mode=1 --test=$test &

SIM_PID=$!
echo "Started ball simulation with PID: $SIM_PID"

# record 10 seconds of data
countdown 10

# Send the SIGINT signal to the rosbag and solver processes
kill -INT "$BAG_PID"
kill -INT "$MPC_PID"
kill -INT "$SIM_PID"

# Close shell
echo "SIGINT sent to PID: $BAG_PID"
echo "SIGINT sent to PID: $MPC_PID"
echo "SIGINT sent to PID: $SIM_PID"
echo "Closing shell."

# Notes:
#	- Convert \motor1_pos to pixels (or metric!) before publishing to make plots more clear (\motor1_pos is in steps currently)
