#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import argparse 

pos = 0
pos_received = False
omega_d = Twist()

# get the position message 
def get_pos(msg): 
	global pos
	global pos_received

	pos = msg.data # converted from steps -> pixels in motor code
	if pos_received == False:
		pos_received = True
		print("Pos "+str(pos)+" received!")
		
if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description="Script that sends omega_d commands to the motor controller to move the players to a desired y-position passed 					from the terminal. (Desired position is in pixels with 0 being furthest from the hardware and 360 being closest.)"
	)
	parser.add_argument("--desired_pos", required=False, type=int, default=0)
	args = parser.parse_args()
	
	desired_pos = args.desired_pos
	
	# initialize the node
	rospy.init_node('mov_agent', anonymous = True)
	pos_sub = rospy.Subscriber("/motor1_pos", Float32, get_pos)
	cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size = 1)
	
	moved = False
	while not moved:
	
		if pos_received:
			while pos != desired_pos:
				#print("[Players are moving to "+str(desired_pos)+"] Moving...")
				if (desired_pos - pos) > 0:
					omega_d.linear.x = 5
				else:
					omega_d.linear.x = -5
				#print(int(((desired_pos - pos)+0.001)/(abs(desired_pos - pos)+0.001)), omega_d.linear.x)
				cmd_pub.publish(omega_d)
			print("[Players are at "+str(desired_pos)+"] Moved.")
			omega_d.linear.x = 0.0
			cmd_pub.publish(omega_d)
			moved = True
