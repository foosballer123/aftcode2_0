#!/bin/bash
# recorder.sh

cd ~/catkin_ws/src/aftcode2_0/csv
echo "Changed directory..."

echo "Starting rosbag..."
rosbag play ~/catkin_ws/src/aftcode2_0/rosbags/$1.bag &

python3 ~/catkin_ws/src/aftcode2_0/scripts/data_agent_2.py --sample_time=60 --file_name=$1.csv $2

#"experiment_5_p45_hv_mode1_test3_1771903734.csv"

#ball_data_1772053873
