import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 
import increase_brightness as ib
import normalizing as norm
import homographic_transformation as ht

import rospy 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 

# Remember: 
# X positions of players are fixed (because they are fixed to a rod) 
# Y positions of players are variable 

img_received = False 

rgb_img = np.zeros((360, 640, 3), dtype = "uint8") 


# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 
  

def detect_corners(image, mask):
	
    img = image.copy()
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = mask.copy()
	
    corners = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=26,
        qualityLevel=0.01,
        minDistance=100,
        blockSize=3,
        useHarrisDetector=False,
        k=0.04
    )
    
    corners = np.intp(corners)  # Convert to integer coords
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(img, (x, y), radius=3, color=(0, 255, 0), thickness=-1)

    return img, gray
    
# hsv filters might need to be adjusted based on lighting conditions
def green_filter(image):

    img = image.copy()
    img_norm = norm.normalize(img)
    img_bright = ib.increase_brightness(img_norm, 30)
	
    wrap = 50
	
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img_bright, cv2.COLOR_BGR2HSV)

	# Green has a Hue of ~60 
    lower_green = np.array([60-wrap, 0, 0])
    upper_green = np.array([60+wrap, 255, 255])
	
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # WARNING: This majorly slows down the code!!
    #for i in range(hsv_masked.shape[0]):
    #    for j in range(hsv_masked.shape[1]):
    #        if hsv_masked[i][j][2] > 0:
    #            print(hsv_masked[i][j])
   	            
    return mask
    
if __name__ == '__main__': 
	
    rospy.init_node('corner_detection', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    corner_pub = rospy.Publisher('/corner_detector', Image, queue_size = 1) 
    gray_pub = rospy.Publisher('/gray_image', Image, queue_size = 1)
    green_pub = rospy.Publisher('/green_image', Image, queue_size = 1)
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
          	
            green = green_filter(frame)
            warped_green = ht.warp(green)
            
            corners, grayscale = detect_corners(frame, warped_green) 
            
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            
            corner_msg = CvBridge().cv2_to_imgmsg(corners, encoding="bgr8")
            gray_msg = CvBridge().cv2_to_imgmsg(grayscale, encoding="8UC1")
            green_msg = CvBridge().cv2_to_imgmsg(green, encoding="8UC1")
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            corner_pub.publish(corner_msg) 
            gray_pub.publish(gray_msg)
            green_pub.publish(green_msg)
            
        rate.sleep()
