#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
import math
import time
import cv2
import numpy as np
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 

img_received = False

x_rod = 490 # position of the rod in the x direction
#x_rod = 40
L = 50 # length of the player in pixels from 'center -> foot'
x_foot = 0 # position of the players foot in pixels
y_torso = 0
dist = 110

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 

def angular_callback(msg):
	global x_foot 
	global x_rod
	
	# offset = math.pi*0.3
	# offset = 3
	# rad = msg.data*((2*math.pi)/400) + offset # steps * rad/step = rad
	x_foot = int(x_rod + L*math.sin(msg.data))

def linear_callback(msg):
	global y_torso
	
	pps = 130 / 525  # pixels per step (it takes 525 steps to move one player ~1/3 of the camera frame)
	y_torso = int(pps * msg.data)  # converting steps into pixels

def warp(image):

    img = image.copy()
	
    # corners of the field in the original image
    pts1 = np.float32([[23,7],[619,12],[19,351],[618,354]])
    # 618 354     19 351
    # mapping to a new image 
    pts2 = np.float32([[0,0],[640,0],[0,360],[640,360]])

    # find the corresponding transformation
    M = cv2.getPerspectiveTransform(pts1,pts2)
    
    # use the transformations to warp the initial image to a new plane/image 
    dst = cv2.warpPerspective(img,M,(640,360))

    return dst
    	
def bound(lower, val, upper):

	if val > upper:
		return upper
	elif val < lower:
		return lower
	else:
		return val
		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('player_encoder_tracking', anonymous = True)
	
	player_linear_sub = rospy.Subscriber("/motor3_pos", Float32, linear_callback, queue_size=10)
	player_angular_sub = rospy.Subscriber("/motor4_rad", Float32, angular_callback, queue_size=10)
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	img_pub = rospy.Publisher('/player_angular_encoder_tracking', Image, queue_size = 1)
	pos_pub = rospy.Publisher('/player_rad_conversion', Float32, queue_size = 1) 
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	while not rospy.is_shutdown():
		
		if img_received: 
			frame = rgb_img 
			frame = warp(frame)
			
			cv2.circle(frame, (x_foot, y_torso), radius=3, color=(0, 255, 0), thickness=-1)
			cv2.circle(frame, (x_foot, y_torso+dist), radius=3, color=(0, 255, 0), thickness=-1)
			cv2.circle(frame, (x_foot, y_torso+2*dist), radius=3, color=(0, 255, 0), thickness=-1)
			
			cv2.circle(frame, (x_rod, y_torso), radius=3, color=(255, 255,  255), thickness=-1)
			cv2.circle(frame, (x_rod, y_torso+dist), radius=3, color=(255, 255, 255), thickness=-1)
			cv2.circle(frame, (x_rod, y_torso+2*dist), radius=3, color=(255, 255, 255), thickness=-1)
			
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
			pos_pub.publish(x_foot)
            
		loop_rate.sleep()
		
		
