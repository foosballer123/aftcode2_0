#!/usr/bin/env python3

# Created by Benjamin Simpson for Capstone II in Spring 2026

# This code is the core logic of the ball simulator which is used to test the MPC Solvers
# This code is the basis for ball_sim_intercept_test.py

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import math
import time
import cv2
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import argparse
from pynput import keyboard
import random

img_received = False
rad_flag = False
pos_flag = False
cmd_recieved = False
omega_d = 0
theta, theta_offset = 0, 0
player_y, player_offset = Float64MultiArray(), 0

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
		
def angular_callback(msg):

	global rad_flag
	global theta
	global theta_offset
	
	theta = msg.data[0] + theta_offset
	if rad_flag == False:
		rad_flag = True
		
def linear_callback(msg):

	global pos_flag
	global player_y
	
	player_y = msg.data 
	if pos_flag == False:
		pos_flag = True		
		
def omega_callback(data):

	global omega_d
	global cmd_recieved
  
	if cmd_recieved == False:
		cmd_recieved = True
 
	omega_d = data.angular.x
	
def offset_callback(msg):

	global theta_offset
	global player_offset
	
	theta_offset = msg.data[0] * math.pi
	player_offset = msg.data[1] 

# x_rod = 0.068

def spawn():
	
	x, vx = random.random() * 0.290 + 0.3, -(random.random() * 1.0)
	y, vy = random.random() * 0.340, random.uniform(-0.5, 0.5)
	
	return [[x, vx], [y, vy]]
		
if __name__ == '__main__':

	parser = argparse.ArgumentParser(
		formatter_class=argparse.RawDescriptionHelpFormatter
	)
	parser.add_argument("--mode", required=False, type=int, default=1)
	parser.add_argument("--test", required=True, type=int, default=1)
	args = parser.parse_args()
	
	mode = args.mode
	test = args.test
	 
	L_P = rospy.get_param('/table_measurements/player_length')
	H_P = rospy.get_param('/table_measurements/player_height')
	R_F = rospy.get_param('/table_measurements/approximate_player_foot_diameter') / 2
	R_B = rospy.get_param('/table_measurements/ball_diameter') / 2

	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/pylon_camera_node/image_rect", Image, get_image)
	img_pub = rospy.Publisher('/simulated_ball_metric', Image, queue_size = 1) 
	ball_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10)
	
	rospy.Subscriber("/omega_d", Twist, omega_callback, queue_size=10)
	rospy.Subscriber("/offset", Float64MultiArray, offset_callback, queue_size=10)
	rospy.Subscriber("/rod2_player_positions", Float64MultiArray, angular_callback, queue_size=10)
	rospy.Subscriber("/rod1_player_positions", Float64MultiArray, linear_callback, queue_size=10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Twist for sending simulated ball positions
	ball_pos = Twist()
	
	init_flag = False
	i = [test, 0]
	v = 0
	
	r0_z = H_P - R_B
	k = rospy.get_param('/x_solver_parameters/k_value')
    
	print("K Value:", k)
    
	def init(test, mode, flag):
	
		
		if (mode == 1) or (mode == 2): 
			if test == 0:
				r0_init = [0.05, 0.170]
				r1_init = [0.0, 0.0]
			if test == 1:
				r0_init = [0.068, 0.170]
				r1_init = [0.0, 0.0]	
			elif test == 2:
				r0_init = [0.068+0.05, 0.170]
				r1_init = [0.0, 0.0]
			elif test == 3:
				r0_init = [0.068+0.1, 0.170]
				r1_init = [0.0, 0.0]
			elif test == 4:
				r0_init = [0.290, 0.170]
				r1_init = [-0.290, 0.0]	
			elif test == 5:
				r0_init = [0.05-0.05, 0.170]
				r1_init = [-0.290, 0.0]	
			elif test == 6:
				r0_init = [0.05-0.1, 0.170]
				r1_init = [-0.290, 0.0]	
				
		if (mode == 3): 
				b = spawn()
				r0_init = [b[0][0], b[1][0]]
				r1_init = [b[0][1], b[1][1]]	
				
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
	
	collisions = []
	
	n = 0
	toggle_flag = 0
	center_flag = 0
	deflect_flag = False
	deflect_ticks = 0
	bounds = 0.01
	
	#while not cmd_recieved:
	#	print("Waiting...")
		
	while not rospy.is_shutdown():
		
		if init_flag == False:
			print("Respawned ball.")
			
			i[0], i[1] = i[0] + 1, i[1] + 1
			
			deflect_flag = False
			deflect_ticks = 0
			
			if mode == 2:
				test = i[0] % 7
				
			print(test)
			r0, r1, dt, init_flag = init(test, mode, init_flag)
			
			if ((mode == 1) or (mode == 3)) and ((i[1] % 2) == 1):
				r0, r1 = [0.585, 0.335], [0, 0] 
				
		if (r0[1] >= (0.340 - bounds)) or (r0[1] <= (0.0 + bounds)):
			r1[1] = -r1[1] # switch direction
		if (r0[0] >= (0.590 - bounds)) or (r0[0] <= (0.0 + bounds)):
			r1[0] = -r1[0] # switch direction
			
		X_ROD = rospy.get_param('/table_measurements/blue_rod_positions/rod_one') + player_offset
		x_foot = X_ROD + L_P*math.sin(theta)
		z_foot = L_P*math.cos(theta)
		
		print(theta, rad_flag)
		
		dist = math.sqrt( (x_foot - r0[0])**2 + (z_foot - r0_z)**2 )
		#collision_gain = math.tanh(200 * max(0, (R_F + R_B) - dist))
		collision_gain = 1 / (1 + math.exp(-300 * ((R_F + R_B) - dist)))
		print(collision_gain)		
		
		r1_x_pre = r1[0]
		 		
		if collision_gain > 0.5:
			print(n, collision_gain)
			r1[0] = (1- collision_gain) * r1[0] + collision_gain * L_P * omega_d * math.cos(theta)
			n += 1
		else:
			n = 0
			
		if collision_gain > 0.95:
			center_flag = 1
		else:
			center_flag = 0	
		
			
		r0[0] = bound(0, r0[0] + r1[0] * dt, 0.590)
		r0[1] = bound(0, r0[1] + r1[1] * dt, 0.340)
		
		if (mode == 3) and ((r1_x_pre + 1e3)/(abs(r1_x_pre) + 1e3)) != ((r1[0] + 1e3)/(abs(r1[0]) + 1e3)):
			deflect_flag = True
		if deflect_flag == True:
			deflect_ticks += 1
		if deflect_ticks >= 30:
			init_flag = False
			
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
			
			r_x = meters_to_pixels(X_ROD, "w")
			f_x = meters_to_pixels(x_foot, "w")
			
			if pos_flag:
				f_y1, f_y2, f_y3 = meters_to_pixels(player_y[0], "h"), meters_to_pixels(player_y[1], "h"), meters_to_pixels(player_y[2], "h")
			
			if n > 0:
				collisions.append((b_x, b_y, collision_gain, center_flag))
				
			cv2.circle(frame, (b_x, b_y), radius=30, color=(255), thickness=-1)
			
			if rad_flag and pos_flag:
				cv2.circle(frame, (r_x, f_y1), radius=5, color=(255), thickness=-1)
				cv2.circle(frame, (r_x, f_y2), radius=5, color=(255), thickness=-1)
				cv2.circle(frame, (r_x, f_y3), radius=5, color=(255), thickness=-1)
				
				cv2.circle(frame, (f_x, f_y1), radius=5, color=(255), thickness=-1)
				cv2.circle(frame, (f_x, f_y2), radius=5, color=(255), thickness=-1)
				cv2.circle(frame, (f_x, f_y3), radius=5, color=(255), thickness=-1)
			
			if len(collisions) > 0:
				for p in collisions:
					cv2.circle(frame, (p[0],p[1]), radius=5, color=(255-int(p[2]*255)), thickness=3)
					
					if p[-1] == 1:
						cv2.circle(frame, (p[0],p[1]), radius=15, color=(255), thickness=1)
						cv2.circle(frame, (p[0],p[1]), radius=50, color=(255), thickness=1)
						
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="8UC1")
			img_pub.publish(img_msg) 
            
		loop_rate.sleep()
		
		
