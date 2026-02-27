#!/bin/bash
# experiment_5_p45_hv_test3.sh


# Check encoder value; if not 1, publish \omega_d until 1.
echo "Waiting for motor 1 to initialize..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/init_agent.py
echo "Initialized."

# Move player to a desired initial position.
echo "Waiting for motor 1 to move..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/mov_agent.py --desired_pos=120
echo "Moved."

# Start MPC Solver node with desired p-horizon (using argparse)
echo "Starting Y-Solver with --prediction_steps=45..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/MPC_Y_Solver.py --prediction_steps=45 & # --> The ampersand '&' ensures that the python scripts runs as a background process so that the next line can execute

# Wait...
sleep 7

# Start recording rosbag with topics \omega_d \ball_pos \motor1_pos \motor2_rad
echo "Recording topics \omega_d \ball_pos \motor1_pos \motor2_rad ..."
rosbag record /camera/color/image_raw /omega_d /ball_pos /motor1_pos -O ~/catkin_ws/src/aftcode2_0/rosbags/experiment_5_p45_hv_mode1_test3_$(date +%s) &

# Start ball simulation node and print 'mode' and 'number'
echo "Starting 'ball_sim_tests.py' with 'mode=1' and 'test=3'..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/ball_sim_tests.py --mode=1 --test=3

# Kill shell on 'ctrl+c'
echo "Killing terminal..."

# Notes:
#	- Convert \motor1_pos to pixels (or metric!) before publishing to make plots more clear (\motor1_pos is in steps currently)
