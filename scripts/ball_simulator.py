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
tvp_received = False
tvp = 0

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
		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	tvp_sub = rospy.Subscriber("/r0_tvp", Float32, get_tvp)
	img_pub = rospy.Publisher('/simulated_ball', Image, queue_size = 1) 
	#img_pub = rospy.Publisher('/player_encoder_tracking', Image, queue_size = 1) 
	# declare a publisher to publish in the velocity command topic
	ball_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10)
	
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Twist for sending simulated ball positions
	ball_pos = Twist()
	
	r0 = 300
	r1 = 600
	dt = 1/60
	
	toggle_flag = 0
	while not rospy.is_shutdown():
		
		if (r0 >= 345) or (r0 <= 15):
			r1 = -r1 # switch direction
			
		r0 = bound(15, r0 + r1 * dt, 345)
		
		ball_pos.linear.x = 200
		ball_pos.angular.x = 0
		ball_pos.linear.y = r0
		ball_pos.angular.y = r1
	
		if ball_pos != None:
			ball_pub.publish(ball_pos)
		
		if img_received: 
			frame = rgb_img 
			
			cv2.circle(frame, (200, int(r0)), radius=15, color=(255, 255, 255), thickness=-1)
			
			if tvp_received:
				cv2.circle(frame, (200, int(tvp)), radius=15, color=(255, 0, 0), thickness=-1)
			
			img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
			img_pub.publish(img_msg) 
            
		loop_rate.sleep()
		
		
