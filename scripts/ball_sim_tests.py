#!/usr/bin/env python3

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

img_received = False
tvp_received = False
tvp = 0
# declare a variable of type Twist for sending simulated ball positions
ball_pos = Twist()

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 
    # get the image message 
    
def get_tvp(msg): 
	global tvp 
	global tvp_received
	tvp = msg.data
	tvp_received = True
  	
def bound(lower, val, upper):

	if val > upper:
		return upper
	elif val < lower:
		return lower
	else:
		return val

def test(mode, number):

	test_mode = mode
	test_number = number	
	
	if test_number == 1:
		r0_init = [620, 46]
		r1_init = [-200,0]
	elif test_number == 2:
		r0_init = [620,20]
		r1_init = [-150*5,-1*-100*5]

	r0 = r0_init.copy()
	r1 = r1_init.copy()
	dt = 1/60

	
	if test_mode == 1:
		if (r0[1] >= 345) or (r0[1] <= 15):
			r1[1] = -r1[1] # switch direction
		if (r0[0] >= 625) or (r0[0] <= 15):
			r1[0] = -r1[0] # switch direction
				
	elif test_mode == 2:	
		if (r0[1] >= 345) or (r0[1] <= 15):
			r1[1] = 0 # switch direction
		if (r0[0] <= 15):
			r1[0] = 0 # switch direction
		if ((r1[0] == 0) and (r1[1] == 0)):
			input("Press enter to reset...")
			r0 = r0_init.copy()
			r1 = r1_init.copy()
			print(r0)
			print(r1)
				
	r0[1] = bound(15, r0[1] + r1[1] * dt, 345)
	r0[0] = bound(15, r0[0] + r1[0] * dt, 625)
		
	return r0, r1
			
if __name__ == '__main__':
	parser = argparse.ArgumentParser(
	description="  *** Script that runs a ball simulation with different modes and tests. *** "
	"\n\n	Mode 1 -- ball bounces indefinitely off of frame boundaries. "
	"\n	Mode 2 -- ball pauses when it encounters frame boundaries and must be reset. "
	"\n\n	Test 1 -- ball spawns center right and slowly moves left. "
	"\n	Test 2 -- ball spawns in top right and slowly moves down and left. "
	"\n	Test 3 -- ball spawns in bottom right and slowly moves up and left. "
	"\n	Test 4 -- ball spawns center right and quickly moves left. "
	"\n	Test 5 -- ball spawns in top right and quickly moves down and left. "
	"\n	Test 6 -- ball spawns in bottom right and quickly moves up and left. "
	"\n	Test 7 -- ball spawns center right and very quickly moves left. "
	"\n	Test 8 -- ball spawns in top right and very quickly moves down and left. "
	"\n	Test 9 -- ball spawns in bottom right and very quickly moves up and left. "
	"\n	Test 10 -- ball spawns close to players and oscillates slowly between walls. "
	"\n	Test 11 -- ball spawns close to players and oscillates quickly between walls. ",
	formatter_class=argparse.RawDescriptionHelpFormatter
	)
	parser.add_argument("--mode", required=True, type=int, default=1)
	parser.add_argument("--test", required=True, type=int, default=1)
	args = parser.parse_args()
	
	mode = args.mode
	test = args.test
    
	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	#tvp_sub = rospy.Subscriber("/r0_tvp", Float32, get_tvp)
	img_pub = rospy.Publisher('/simulated_ball', Image, queue_size = 1) 
	#img_pub = rospy.Publisher('/player_encoder_tracking', Image, queue_size = 1) 
	# declare a publisher to publish in the velocity command topic
	ball_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	test_mode = mode
	test_number = test	
	scale_1 = 3
	scale_2 = 7
	
	# Test 1 -- ball spawns center right and slowly moves left 
	if test_number == 1:
		r0_init = [580, 180]
		r1_init = [int(16/9)*(-100),0]
	# Test 2 -- ball spawns in top right and slowly moves down and left 
	elif test_number == 2:
		r0_init = [580,20]
		r1_init = [int((16/9)*(-100)),100]
	# Test 3 -- ball spawns in bottom right and slowly moves up and left 
	elif test_number == 3:
		r0_init = [580,340]
		r1_init = [int((16/9)*(-100)),-100]
	# Test 4 -- ball spawns center right and quickly moves left 
	if test_number == 4:
		r0_init = [580, 180]
		r1_init = [int(16/9)*(-100)*scale_1,0*scale_1]
	# Test 5 -- ball spawns in top right and quickly moves down and left 
	elif test_number == 5:
		r0_init = [580,20]
		r1_init = [int((16/9)*(-100))*scale_1,100*scale_1]
	# Test 6 -- ball spawns in bottom right and quickly moves up and left 
	elif test_number == 6:
		r0_init = [580,340]
		r1_init = [int((16/9)*(-100))*scale_1,-100*scale_1]	
	# Test 7 -- ball spawns center right and very quickly moves left 
	if test_number == 7:
		r0_init = [580, 180]
		r1_init = [int(16/9)*(-100)*scale_2,0*scale_2]
	# Test 8 -- ball spawns in top right and very quickly moves down and left 
	elif test_number == 8:
		r0_init = [580,20]
		r1_init = [int((16/9)*(-100))*scale_2,100*scale_2]
	# Test 9 -- ball spawns in bottom right and very quickly moves up and left 
	elif test_number == 9:
		r0_init = [580,340]
		r1_init = [int((16/9)*(-100))*scale_2,-100*scale_2]	
	# Test 9 -- ball spawns in bottom right and very quickly moves up and left 
	elif test_number == 9:
		r0_init = [580,340]
		r1_init = [int((16/9)*(-100))*scale_2,-100*scale_2]
	# Test 10 -- ball spawns close to players and oscillates slowly between walls
	elif test_number == 10:
		r0_init = [190,180]
		r1_init = [0,-100]
	# Test 11 -- ball spawns close to players and oscillates quickly between walls			
	elif test_number == 11:
		r0_init = [190,180]
		r1_init = [0,-100*scale_1]	
		
	r0 = r0_init.copy()
	r1 = r1_init.copy()
	dt = 1/60
	
	while not rospy.is_shutdown():
		
		# Mode 1 -- ball bounces indefinitely off of frame boundaries
		if test_mode == 1:
			if (r0[1] >= 345) or (r0[1] <= 15):
				r1[1] = -r1[1] # switch direction
			if (r0[0] >= 625) or (r0[0] <= 15):
				r1[0] = -r1[0] # switch direction
		# Mode 2 -- ball pauses when it encounters frame boundaries and must be reset		
		elif test_mode == 2:	
			if (r0[1] >= 345) or (r0[1] <= 15):
				r1[1] = 0 # switch direction
			if (r0[0] <= 15):
				r1[0] = 0 # switch direction
			if ((r1[0] == 0) and (r1[1] == 0)):
				input("Press enter to reset...")
				r0 = r0_init.copy()
				r1 = r1_init.copy()
				print(r0)
				print(r1)
				
		r0[1] = bound(15, r0[1] + r1[1] * dt, 345)
		r0[0] = bound(15, r0[0] + r1[0] * dt, 625)
			
		ball_pos.linear.x = r0[0]
		ball_pos.angular.x = r1[0]
		ball_pos.linear.y = r0[1]
		ball_pos.angular.y = r1[1]
	
		if ball_pos != None:
			ball_pub.publish(ball_pos)
		
		if img_received: 
			frame = rgb_img 
			
			cv2.circle(frame, (int(r0[0]), int(r0[1])), radius=15, color=(255, 255, 255), thickness=-1)
				
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
			
		loop_rate.sleep()
