#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import GOOD_DEFENSE as DEFENSE

img_received = False

rgb_img = np.zeros((360, 640, 3), dtype = "uint8")

ball_pos = Twist() #geometry position with XYZ for linear and angular


#--------------------------------------------------------------------------------
#img = cv2.imread('C:/Users/benki/Documents/School/Programming/Blue Team/Data/colorful_circles.jpg', 0)
#img = cv2.imread('C:/Users/benki/Documents/School/Programming/Blue Team/Data/stockFoosball.jpg', 0) # Replace with your image path
# cap = cv2.VideoCapture("17output.mp4")
#cap = cv2.VideoCapture(1)

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
#--------------------------------------------------------------------------------


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True
	
	
	
# Set Hough Circle Transform parameters
min_dist = 50  # Minimum distance between detected circles
param1 = 50   # Canny edge detection threshold
param2 = 30   # Acumulator threshold for circle detection
min_radius = 8 # Minimum circle radius
max_radius = 20 # Maximum circle radius

if __name__ == '__main__':

	rospy.init_node('ROBOT_EYES_AND_HANDS', anonymous = True)
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
	img_pub = rospy.Publisher('/eyes', Image, queue_size = 1)
	
	pos_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10) 
	
	rate = rospy.Rate(30)
	
	print("Run")
	
	while not rospy.is_shutdown():
	    
		if img_received:


    			#ret, frame = cap.read()
    			#if not ret:
        			#break
        		
			frame = rgb_img	
        		

    			# Convert frame to grayscale
			gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    			# Apply median blur to reduce noise
    			#gray_frame = cv2.medianBlur(gray_frame, 1)

    			# Detect circles
			circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT, dp=1, minDist=min_dist, param1=param1, param2=param2, minRadius=min_radius, maxRadius=max_radius)

			if circles is not None:
				# Convert the (x, y) coordinates and radius to integers
				circles = np.uint16(np.around(circles))

				
        			# Draw the detected circle
        			# for i in circles[0, :]:
        			#     center = (i[0], i[1])
        			#     radius = i[2]
        			#     cv2.circle(gray_frame, center, radius, (255, 0, 255), 3)
				#print("There it is!")
				for i in circles[0]:
				
					ball_pos.angular.x = 1 #Is the ball in frame?
					
					center = (i[0], i[1])
					
					ball_pos.linear.x = i[0]
					ball_pos.linear.y = i[1]
					ball_pos.linear.z = DEFENSE.defense(center[1])
					
					radius = i[2]
					cv2.circle(gray_frame, center, radius, (255, 0, 255), 3)

			else:
				#print("Where'd it go man!")
				ball_pos.angular.x = 0 # Did the ball dissapear 
			
        			# Show the image with the detected circle
        			#cv2.imshow('Detected Circle', gray_frame)
        			#cv2.waitKey(25)
        			#cv2.destroyAllWindows()

				#print(center)
			
			#cv2.imshow('Detected Circle', gray_frame)

			img_msg = CvBridge().cv2_to_imgmsg(gray_frame, encoding="8UC1")
			img_pub.publish(img_msg)
			pos_pub.publish(ball_pos)        			
		rate.sleep()		
        			
        			
