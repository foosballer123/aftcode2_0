#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import math

img_received = False

mono_img = np.zeros((720, 1280, 1), dtype = "uint8") # Automated this !!!

ball_pos = Twist()  # geometry position with XYZ for linear and angular

# get the image message
def get_image(ros_img):
    global mono_img
    global img_received
    # convert to opencv image
    mono_img = CvBridge().imgmsg_to_cv2(ros_img, "8UC1")
    # raise flag
    img_received = True
    
    
def create_white_mask(frame):
    
    lower_white = np.array([180])
    upper_white = np.array([255])
    white_mask = cv2.inRange(frame, lower_white, upper_white)

    return white_mask

def filter_with_opening_and_growth(binary_mask, kernel_size=7, growth_iterations=2):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    opened_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
    growth_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    grown_mask = cv2.dilate(opened_mask, growth_kernel, iterations=growth_iterations)
    return grown_mask

def detect_blobs(mask):
    detections = 0
    position = None

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 3.14 * 12 * 12 # Adjust based on resolution... Automated this !!!
    params.maxArea = 3.14 * (20*4) * (20*4)
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.maxCircularity = 1.0
    params.filterByConvexity = True
    params.minConvexity = 0.85
    params.maxConvexity = 1.0
    params.filterByInertia = True
    params.minInertiaRatio = 0.5
    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(mask)

    if keypoints:
        largest_keypoint = max(keypoints, key=lambda kp: kp.size)
        position = (int(largest_keypoint.pt[0]), int(largest_keypoint.pt[1]))
        detections = 1

    return position, detections

def detect_hough_circles(mask, frame):
    detections = 0
    position = None

    blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2)
    circles = cv2.HoughCircles(blurred_mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                               param1=50, param2=15, minRadius=10, maxRadius=20)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            radius = i[2]
            if 10 <= radius <= 20:
                position = (i[0], i[1])
                detections = 1
                break

    return position, detections

def clamp_position_to_board(position, board_bounds):
    x_min, y_min, x_max, y_max = board_bounds
    x = max(x_min, min(x_max, position[0]))
    y = max(y_min, min(y_max, position[1]))
    return (x, y)

if __name__ == '__main__':

    rospy.init_node('computer_vision_mono8', anonymous = True)
    img_sub = rospy.Subscriber("/pylon_camera_node/image_rect", Image, get_image)
    img_pub = rospy.Publisher('/computer_vision_mono8', Image, queue_size = 1)
    mask_pub = rospy.Publisher('/computer_vision_mono8_mask', Image, queue_size = 1)
    pos_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10) 
    
    field_height = rospy.get_param('/table_measurements/field_height')
    field_width = rospy.get_param('/table_measurements/field_width')
    
    meters_per_pixel_x = field_height / 720
    meters_per_pixel_y = field_width / 1280
    
    rate = rospy.Rate(60)

    print("Run")

    board_bounds = (0, 0, 1279, 719)  # Automated this !!!
    
    time, last_time = rospy.get_time(), 0.001
    last_position = [0, 0]
    
    while not rospy.is_shutdown():
		
        if img_received:
            frame = mono_img
           
            white_mask = create_white_mask(frame)
            filtered_mask = filter_with_opening_and_growth(white_mask, kernel_size=11, growth_iterations=2)
            #filtered_bgr = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)

            position, detections = detect_blobs(filtered_mask)

            if position is not None:
                position, time = clamp_position_to_board(position, board_bounds), rospy.get_time()
                
            elif last_position is not None:
                position, time = last_position, last_time
			   	
            if detections != 0:
                cv2.circle(frame, position, (15*2), (100), 3) # Adjust circle size based on resolution! 
            
            img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="8UC1")
            mask_msg = CvBridge().cv2_to_imgmsg(filtered_mask, encoding="8UC1")
            
            dt = 1/ 60 # (time - last_time)
            
            if position is not None and last_position is not None:
                ball_pos.linear.x, ball_pos.linear.y = (position[0] * meters_per_pixel_x), (position[1] * meters_per_pixel_y)
                ball_pos.angular.x, ball_pos.angular.y = (last_position[0] - position[0]) / dt, (last_position[1] - position[1]) / dt
            
            if position is not None:
            	last_position, last_time = position, time
    
            mask_pub.publish(mask_msg)
            img_pub.publish(img_msg)
            pos_pub.publish(ball_pos)
            
        rate.sleep()

