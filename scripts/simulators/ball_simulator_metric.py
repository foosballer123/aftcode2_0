#!/usr/bin/env python3

# Created by Benjamin Simpson for Capstone II in Spring 2026

# This code is the core logic of the ball simulator which is used to test the MPC Solvers
# This code is the basis for ball_sim_intercept_test.py

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
import math
import time
import cv2
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import argparse
from pynput import keyboard

img_received = False

# get the image message 
def get_image(ros_img): 
    global mono8_img  
    global img_received 
    # convert to opencv image 
    mono8_img = CvBridge().imgmsg_to_cv2(ros_img, "8UC1") 
    # raise flag 
    img_received = True 
    # get the image message 
    
def bound(lower, val, upper):

	if val > upper:
		return upper
	elif val < lower:
		return lower
	else:
		return val

def meters_to_pixels(x, dimension):

	if dimension == "w":
		return int(x * (1280/0.590))
	elif dimension == "h":
		return int(x * (720/0.340))
	else:
		return 0
		
		
if __name__ == '__main__':

	parser = argparse.ArgumentParser(
		formatter_class=argparse.RawDescriptionHelpFormatter
	)
	parser.add_argument("--mode", required=False, type=int, default=1)
	parser.add_argument("--test", required=True, type=int, default=1)
	args = parser.parse_args()
	
	mode = args.mode
	test = args.test
	
	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/pylon_camera_node/image_rect", Image, get_image)
	img_pub = rospy.Publisher('/simulated_ball_metric', Image, queue_size = 1) 
	ball_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Twist for sending simulated ball positions
	ball_pos = Twist()
	
	init_flag = False
	i = [test, 0]
	
	def init(test, flag):
	
		if test == 0:
			r0_init = [0.068, 0.170]
			r1_init = [0.0, 0.0]	
		elif test == 1:
			r0_init = [0.068+0.05, 0.170]
			r1_init = [0.0, 0.0]
		elif test == 2:
			r0_init = [0.068+0.1, 0.170]
			r1_init = [0.0, 0.0]
		elif test == 3:
			r0_init = [0.290, 0.170]
			r1_init = [-0.290, 0.0]	

		r0 = r0_init.copy()
		r1 = r1_init.copy()
		dt = 1/60
		
		return r0, r1, dt, True
		
	def on_press(key):
		global init_flag
		try:
			if key.char == 'q':
				print("Respawning ball...")
				init_flag = False
		except AttributeError:
			pass  # Handles special keys

	# Start listener in background thread
	listener = keyboard.Listener(on_press=on_press)
	listener.start()

	toggle_flag = 0
	while not rospy.is_shutdown():
		
		if init_flag == False:
			print("Respawned ball.")
			
			i[0], i[1] = i[0] + 1, i[1] + 1
			
			if mode == 2:
				test = i[0] % 4
				
			print(test)
			r0, r1, dt, init_flag = init(test, init_flag)
			
			if (mode == 1) and ((i[1] % 2) == 1):
				r0, r1 = [0.585, 0.335], [0, 0] 
				
		if (r0[1] >= (0.340 - 0.05)) or (r0[1] <= (0.0 + 0.05)):
			r1[1] = -r1[1] # switch direction
		if (r0[0] >= (0.590 - 0.05)) or (r0[0] <= (0.0 + 0.05)):
			r1[0] = -r1[0] # switch direction
			
		r0[0] = bound(0, r0[0] + r1[0] * dt, 0.590)
		r0[1] = bound(0, r0[1] + r1[1] * dt, 0.340)
		
		ball_pos.linear.x = r0[0]
		ball_pos.linear.y = r0[1]
		ball_pos.angular.x = r1[0]
		ball_pos.angular.y = r1[1]
		
		if ball_pos != None:
			ball_pub.publish(ball_pos)
		
		if img_received: 
			frame = mono8_img 
			
			
			b_x = meters_to_pixels(r0[0], "w")
			b_y = meters_to_pixels(r0[1], "h")
			
			cv2.circle(frame, (b_x, b_y), radius=30, color=(255), thickness=-1)
				
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="8UC1")
			img_pub.publish(img_msg) 
            
		loop_rate.sleep()
		
		
