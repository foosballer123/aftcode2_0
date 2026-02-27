#!/bin/bash
# observer.sh

# Starting python script that prints lines to terminal
echo "Starting 'agent.py'..."
python3 ~/catkin_ws/src/aftcode2_0/scripts/init_agent.py
echo "Finished python."

echo "Starting loop. Type in the terminal and it will print whatever you say."
# Read lines print by 'agent.py'
while read -r line; do
	echo "Got: $line"
done
echo "Finished loop."
