#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
import math
import time
import casadi as cs
import cv2
from std_msgs.msg import Float32
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import numpy as np

img_received = False
cmd_received = False
omega_d = Twist()

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 
    # get the image message 

def get_cmd(msg):
    global cmd_received
    global omega_d
    
    omega_d = msg.linear.y
    
    if cmd_received == False:
        cmd_received = True
        
def bound(lower, val, upper):

	if val > upper:
		return upper
	elif val < lower:
		return lower
	else:
		return val
		
def warp(image):

    # corners of the field in the original image
    pts1 = np.float32([[25,7],[616,12],[23,351],[618,352]])
    
    # mapping to a new image 
    pts2 = np.float32([[0,0],[640,0],[0,360],[640,360]])

    # find the corresponding transformation
    M = cv2.getPerspectiveTransform(pts1,pts2)
    
    # use the transformations to warp the initial image to a new plane/image 
    image = cv2.warpPerspective(image,M,(640,360))

    return image
    		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	cmd_sub = rospy.Subscriber("omega_d", Twist, get_cmd)
	img_pub = rospy.Publisher('/simulated_player', Image, queue_size = 1) 
	player_pub = rospy.Publisher('/player_angular_pos', Float32, queue_size = 10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Float32 for sending simulated player positions
	player_pub = 0.0
	
	offset = 452
	theta = 0
	x_rod = 35 + offset
	l = 50
	dt = 1/60
	
	while not rospy.is_shutdown():
		
		if img_received: 
			frame = rgb_img 
			frame = warp(frame)
			
			if cmd_received:
			    theta = theta + omega_d * dt
			    x_foot = x_rod + l*cs.sin(theta)
			  
			    print("Theta:", theta)
			    print("Foot:", x_foot)
			    
			    input()
			    cv2.circle(frame, (int(x_foot), 180), radius=5, color=(255, 255, 255), thickness=-1)
			    #cv2.circle(frame, (int(x_rod), 180), radius=5, color=(255, 255, 255), thickness=-1)
			    
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
            
		loop_rate.sleep()
		
		
