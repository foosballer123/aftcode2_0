import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 
import increase_brightness as ib

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
  
def mask_field(frame): 

    h, w = frame.shape[:2] 
   
    # create a mask that will be used to isolate the players 
    mask = np.zeros((h, w), dtype=np.uint8) 
   	
    mask_table = mask.copy() 
       
    # mask to remove the borders of the table (needs to be adjusted with camera frame) 
    x1, y1, x2, y2 = (0, 12, 615, 355) 
    cv2.rectangle(mask_table, (x1, y1), (x2, y2), color=255, thickness=-1) 
    frame = cv2.bitwise_and(frame, frame, mask=mask_table) 
   
    return frame


# hsv filters might need to be adjusted based on lighting conditions
def hsv_filter(image):

    img = image
    
    B, G, R = cv2.split(img)
    
    R = (((R-R.min())/(R.max()-R.min()))*255.0).astype(np.uint8)
    G = (((G-G.min())/(G.max()-G.min()))*255.0).astype(np.uint8)
    B = (((B-B.min())/(B.max()-B.min()))*255.0).astype(np.uint8)
    
    img = cv2.merge([B, G, R])
    
    img = ib.increase_brightness(img, 30)
    
    red_bounds = 1
    wrap = 15
	
	
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    if red_bounds == 1:
        # Define the range for the red color in HSV
        lower_red1 = np.array([0, 200, 50])
        upper_red1 = np.array([wrap, 255, 255])
        lower_red2 = np.array([180-wrap, 200, 50])
        upper_red2 = np.array([180, 255, 255])
	
        # Create masks for the red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
	
    if red_bounds == 0:
	    lower_red = np.array([0, 225, 200])
	    upper_red = np.array([1, 255, 255])
	
	    mask = cv2.inRange(hsv, lower_red, upper_red)
	    
    kernel = np.ones((5, 5), np.uint8)
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    
    red_masked = cv2.bitwise_and(img, img, mask=mask_dilation)
    hsv_masked = cv2.bitwise_and(hsv, hsv, mask=mask_dilation)
    
    # WARNING: This majorly slows down the code!!
    #for i in range(hsv_masked.shape[0]):
   	#    for j in range(hsv_masked.shape[1]):
   	#        if hsv_masked[i][j][2] > 0:
   	#            print(hsv_masked[i][j])
   	            
    return hsv_masked, red_masked
    
if __name__ == '__main__': 
	
    rospy.init_node('hsv_test', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    hsv_pub = rospy.Publisher('/hsv_filter', Image, queue_size = 1) 
    red_pub = rospy.Publisher('/red_filter', Image, queue_size = 1) 
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
          
            field = mask_field(frame) 
            hsv, red = hsv_filter(field)
            
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            hsv_msg = CvBridge().cv2_to_imgmsg(hsv, encoding="bgr8") 
            red_msg = CvBridge().cv2_to_imgmsg(red, encoding="bgr8")
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            hsv_pub.publish(hsv_msg) 
            red_pub.publish(red_msg)
            
        rate.sleep()
