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
motor1_pos = 0

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 

def player_callback(msg):
	global motor1_pos
	
	pps = 130 / 525  # pixels per step (it takes 525 steps to move one player ~1/3 of the camera frame)
	motor1_pos = pps * msg.data  # converting steps into pixels
	
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
	
	player_sub = rospy.Subscriber("/motor1_pos", Float32, player_callback, queue_size=10)
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	img_pub = rospy.Publisher('/player_encoder_tracking', Image, queue_size = 1)
	pos_pub = rospy.Publisher('/player_position_conversion', Float32, queue_size = 1) 
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	dist = 112
	rod = 40
	while not rospy.is_shutdown():
		
		if img_received: 
			frame = rgb_img 
			frame = warp(frame)
			
			cv2.circle(frame, (rod, 0), radius=3, color=(0, 0, 255), thickness=-1)
			cv2.circle(frame, (rod, int(motor1_pos)), radius=3, color=(255, 255, 255), thickness=-1)
			cv2.circle(frame, (rod, int(motor1_pos)+dist), radius=3, color=(255, 255, 255), thickness=-1)
			cv2.circle(frame, (rod, int(motor1_pos)+2*dist), radius=3, color=(255, 255, 255), thickness=-1)
			
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
			pos_pub.publish(motor1_pos)
            
		loop_rate.sleep()
		
		
