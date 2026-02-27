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

img_received = False

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
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
		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	img_pub = rospy.Publisher('/simulated_player', Image, queue_size = 1) 
	player_pub = rospy.Publisher('/player_angular_pos', Float32, queue_size = 10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Float32 for sending simulated player positions
	player_pub = 0.0
	
	toggle_flag = 0
	while not rospy.is_shutdown():
		
		if img_received: 
			frame = rgb_img 
			
			cv2.circle(frame, (int(r0[0]), int(r0[1])), radius=15, color=(255, 255, 255), thickness=-1)
			
			#if tvp_received:
			#	cv2.circle(frame, (200, int(tvp)), radius=15, color=(255, 0, 0), thickness=-1)
			
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
            
		loop_rate.sleep()
		
		
