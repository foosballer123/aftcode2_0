#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import argparse 
import csv
import math

"""
Script that subscribes to rostopics /omega_d /ball_pos /motor1_pos and records them to a csv file.

	This script is meant to be used in tandem with rosbags (found in the rosbags directory).
	
	This script:
	
		1) Waits for a first message to be published to a rostopic
		2) Starts recording at a user defined sample time (default=60HZ)
		
"""

# Defining data variables and flags
player_pos = 0
ball_pos = [[0,0],[0,0]] # [position (x,y), velocity (x,y)]
omega_d = Twist()
pos_received = False
ball_received = False
cmd_received = False

# get the position message and set flag
def get_pos(msg): 
	global player_pos
	global pos_received

	player_pos = msg.data # converted from steps -> pixels in motor code
	
	if pos_received == False:
		pos_received = True
		print("Pos "+str(player_pos)+" received!")
		
# get the ball position message and set flag
def get_ball(msg): 
	global ball_pos
	global ball_received

	ball_pos = [[msg.linear.x, msg.linear.y], [msg.angular.x, msg.angular.y]]
	if ball_received == False:
		ball_received = True
		print("Ball Pos "+str(ball_pos)+" received!")	

# get the cmd message and set flag
def get_cmd(msg): 
	global omega_d
	global cmd_received
	
	pps = 120 / 525 # pixels per step
	rpp = math.pow(pps,-1) * (2*math.pi / 400) # radians per pixel
	
	omega_d = msg.linear.x / rpp # *rpp in solver
	
	if cmd_received == False:
		cmd_received = True
		print("Cmd "+str(omega_d)+" received!")
			
if __name__ == '__main__':
	parser = argparse.ArgumentParser(
		description="Script that listens to ros topics and records data to a csv file with a user defined sample time."
	)
	parser.add_argument("--sample_time", required=False, type=int, default=60)
	parser.add_argument("--file_name", required=True, type=str)
	args = parser.parse_args()
	
	sample_time = args.sample_time
	file_name = args.file_name
	
	# initialize the node
	rospy.init_node('data_agent', anonymous = True)
	pos_sub = rospy.Subscriber("/motor1_pos", Float32, get_pos)
	ball_sub = rospy.Subscriber("/ball_pos", Twist, get_ball)
	cmd_sub = rospy.Subscriber("/omega_d", Twist, get_cmd)
	
	rate = rospy.Rate(sample_time)
	
	header = False
	repeat = 0
	p_data = []
	t_s = rospy.Time.now()
	with open(file_name, 'w', newline='') as csvfile:
	
		spamwriter = csv.writer(csvfile, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
		while not rospy.is_shutdown():
	
			if cmd_received and ball_received and pos_received:
			
				t_n = rospy.Time.now()
				t_e = (t_n - t_s).to_sec()
				data = [ball_pos[0][0], ball_pos[0][1], ball_pos[1][0], ball_pos[1][1], player_pos, omega_d, t_e]
				
				if data[:(len(data)-1)] == p_data:
					repeat += 1
				else:
					repeat = 0
					
				if not header:
					spamwriter.writerow(['ball_x','ball_y','ball_vx','ball_vy','player_y1','omega_d','time'])
					print("Wrote:", ['ball_x','ball_y','ball_vx','ball_vy','player_y1','omega_d','time'])
					header = True
					
				spamwriter.writerow(data)
				print("Wrote:", data)
			
				p_data = data[:(len(data)-1)]
				
				if repeat == 10:
					break
					
			rate.sleep()
		
