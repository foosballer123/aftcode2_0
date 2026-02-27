#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import GOOD_DEFENSE as DEFENSE
import math

img_received = False

rgb_img = np.zeros((360, 640, 3), dtype = "uint8")

ball_pos = Twist()  # geometry position with XYZ for linear and angular

# get the image message
def get_image(ros_img):
    global rgb_img
    global img_received
    # convert to opencv image
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
    # raise flag
    img_received = True
    
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
    
def create_white_mask_hsv(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 70, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 1))
    blue_mask_expanded = cv2.dilate(blue_mask, horizontal_kernel, iterations=1)

    white_mask_cleaned = cv2.bitwise_and(white_mask, cv2.bitwise_not(blue_mask_expanded))

    return white_mask_cleaned

def filter_with_opening_and_growth(binary_mask, kernel_size=7, growth_iterations=2):
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    opened_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
    growth_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    grown_mask = cv2.dilate(opened_mask, growth_kernel, iterations=growth_iterations)
    return grown_mask

def detect_blobs(mask, frame):
    detections = 0
    position = None

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 3.14 * 12 * 12
    params.maxArea = 3.14 * 20 * 20
    params.filterByCircularity = True
    params.minCircularity = 0.8
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

    rospy.init_node('ball_tracking_and_estimation', anonymous = True)
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
    img_pub = rospy.Publisher('/ball_tracking_and_estimation', Image, queue_size = 1)
    mask_pub = rospy.Publisher('/ball_tracking_and_estimation_mask', Image, queue_size = 1)
    pos_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10) 
    
    rate = rospy.Rate(60)

    print("Run")

    last_position = None
    last_velocity = (0, 0)
    last_gray = None
    board_bounds = (0, 0, 639, 359)  # Add your actual board bounds if different
    pause = 0
    i = 0
	
    while not rospy.is_shutdown():

        if img_received:
            frame = rgb_img
            frame = warp(frame) # warping frame to match 'coordinate_pipeline.py'
            
            velocity = (0, 0)
            i += 1
           
            white_mask = create_white_mask_hsv(frame)
            filtered_mask = filter_with_opening_and_growth(white_mask, kernel_size=7, growth_iterations=2)
            filtered_bgr = cv2.cvtColor(filtered_mask, cv2.COLOR_GRAY2BGR)
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            position, detections = detect_blobs(filtered_mask, frame)
            detection_method = "Blob Detection"

            if detections == 0:
                position, detections = detect_hough_circles(filtered_mask, frame)
                detection_method = "Hough Circle Detection"

            if detections == 0 and last_position is not None and last_gray is not None:
                p0 = np.array([[last_position]], dtype=np.float32)
                p1, st, err = cv2.calcOpticalFlowPyrLK(last_gray, frame_gray, p0, None)
                if st[0][0] == 1:
                    flow_pos = (int(p1[0][0][0]), int(p1[0][0][1]))
                    
                    # pred_pos = last position + time the ball traveled between frames
                    print("Last position + last velocity (x_n-1,v_x_n-1,y_n-1,v_y_n-1) "+str(i)+":", last_position[0], last_velocity[0], last_position[1], last_velocity[1])
                    pred_pos = (int(last_position[0]) + int(last_velocity[0]), int(last_position[1]) + int(last_velocity[1]))
                    vx, vy = last_velocity
                    speed = math.hypot(vx, vy)
                    if speed > 0:
                        position = (int(0 * flow_pos[0] + 1 * pred_pos[0]),
                                    int(0 * flow_pos[1] + 1 * pred_pos[1]))
                        print("Super fast!! WOW!")
                    #else:
                    #    position = (int(flow_pos[0]), int(flow_pos[1]))
                    #    print("Using flow B-)")
                    detections = 2
                    detection_method = "Optical Flow + Prediction"
                else:
                    position = (int(last_position[0]) + int(last_velocity[0]), int(last_position[1]) + int(last_velocity[1]))
                    detections = 3
                    detection_method = "Fallback Prediction"

            if position is not None:
                position = clamp_position_to_board(position, board_bounds)
			
			# velocity = (dx / time between frames, dy / time between frames ) in pixels per second
            if position is not None and last_position is not None:
                print("Last positions (x_n,x_n-1,y,y_n-1) "+str(i)+":", position[0], last_position[0], position[1], last_position[1])
                velocity = (int(position[0]) - int(last_position[0]), int(position[1]) - int(last_position[1]))
			
            #if (last_position != None) and ( abs(position[0] - last_position[0]) < 10 ):
            #    position = last_position
			
            if position is not None and ((position[0] < 10 or position[0] > 629) or (position[1] < 10 or position[1] > 349)):
                position = (320, 180)
                velocity = (0, 0)
                last_velocity = (0, 0)
                last_position = None
                detection = 0
                last_gray = None
                speed = 0
                
            last_velocity = velocity if position != last_position else last_velocity
            last_position = position
            last_gray = frame_gray.copy()
			
            if position is not None:
                ball_pos.linear.x = position[0]
                ball_pos.linear.y = position[1]
                
                ball_pos.angular.x = velocity[0]
                ball_pos.angular.y = velocity[1]
                
                if ((abs(velocity[0]/(1/60)) <= 1000) and (abs(velocity[1]/(1/60)) <= 1000)):
                    #ball_pos.linear.z = DEFENSE.defense(position[1])
                    ball_pos.linear.z = (velocity[0]/(1/60)) # distance traveled / time between frames in pix/sec 
                    ball_pos.angular.z = (velocity[1]/(1/60)) # distance traveled / time between frames in pix/sec 
                
                if detection_method == "Blob Detection":
                    cv2.circle(frame, position, 15, (255, 0, 255), 3)
                if detection_method == "Hough Circle Detection":
                    cv2.circle(frame, position, 15, (0, 255, 0), 3)
                if detection_method == "Optical Flow + Prediction":
                    cv2.circle(frame, position, 15, (255, 0, 0), 3)
                #print(velocity[0], velocity[1])
                
            img_msg = CvBridge().cv2_to_imgmsg(frame, encoding="rgb8")
            mask_msg = CvBridge().cv2_to_imgmsg(filtered_mask, encoding="8UC1")
      
            img_pub.publish(img_msg)
            mask_pub.publish(mask_msg)
            pos_pub.publish(ball_pos)
            
        rate.sleep()

