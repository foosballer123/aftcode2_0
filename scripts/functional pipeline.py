import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 

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
    
   
# hsv filters might need to be adjusted based on lighting conditions
def normalize(image):

    img = image
    
    B, G, R = cv2.split(img)
    
    R = (((R-R.min())/(R.max()-R.min()))*255.0).astype(np.uint8)
    G = (((G-G.min())/(G.max()-G.min()))*255.0).astype(np.uint8)
    B = (((B-B.min())/(B.max()-B.min()))*255.0).astype(np.uint8)
    
    img = cv2.merge([B, G, R])
    
    return img
    
    
# hsv filters might need to be adjusted based on lighting conditions
def warp(image):

    img = image
	
    # corners of the field in the original image
    pts1 = np.float32([[23,7],[619,12],[19,351],[618,354]])
 
    # mapping to a new image 
    pts2 = np.float32([[0,0],[640,0],[0,360],[640,360]])

    # find the corresponding transformation
    M = cv2.getPerspectiveTransform(pts1,pts2)
    
    # use the transformations to warp the initial image to a new plane/image 
    dst = cv2.warpPerspective(img,M,(640,360))
    
    return dst
    
def detect_corners(image, mask):
	
    img = image
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = mask
	
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
def green_filter(image, num):

    img = image

    wrap = num
	
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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

# hsv filters might need to be adjusted based on lighting conditions
def hsv_filter(image, flag, num, size):

    img = image
    
    wrap_around = flag # 0 or 1
    wrap = num
	
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    if wrap_around == 1:
        # Define the range for the red color in HSV
        lower_red1 = np.array([0, 200, 50])
        upper_red1 = np.array([wrap, 255, 255])
        lower_red2 = np.array([180-wrap, 200, 50])
        upper_red2 = np.array([180, 255, 255])
	
        # Create masks for the red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
	
    if wrap_around == 0:
	    lower_red = np.array([0, 225, 200])
	    upper_red = np.array([1, 255, 255])
	
	    mask = cv2.inRange(hsv, lower_red, upper_red)
	    
    kernel = np.ones((size, size), np.uint8)
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)
    
    # WARNING: This majorly slows down the code!!
    #for i in range(hsv_masked.shape[0]):
   	#    for j in range(hsv_masked.shape[1]):
   	#        if hsv_masked[i][j][2] > 0:
   	#            print(hsv_masked[i][j])
   	            
    return mask_dilation
    
# hsv filters might need to be adjusted based on lighting conditions
def increase_brightness(image, num):

    img = image #.copy()
    
    B, G, R = cv2.split(img)
    
    def increase_value(input_value, increase_by):
        if input_value + increase_by > 255:
            return 255
        else:
            return input_value + increase_by

    new_B = [increase_value(v,num) for v in B.flatten()]
    # using dtype=uint8 as dtype
    new_B = np.array(new_B, dtype=np.uint8)
    new_B = new_B.reshape(B.shape)

    new_G = [increase_value(v,num) for v in G.flatten()]
    # using dtype=uint8 as dtype
    new_G = np.array(new_G, dtype=np.uint8)
    new_G = new_G.reshape(G.shape)
    
    new_R = [increase_value(v,num) for v in R.flatten()]
    # using dtype=uint8 as dtype
    new_R = np.array(new_R, dtype=np.uint8)
    new_R = new_R.reshape(R.shape)
    
    img = cv2.merge([new_B, new_G, new_R])
    
    return img
    
       
if __name__ == '__main__': 
	
    rospy.init_node('functional_pipeline', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    pipe_pub = rospy.Publisher('/functional_pipeline', Image, queue_size = 1) 
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
            
            normalized_frame = normalize(frame)
            bright_frame = increase_brightness(normalized_frame, 30)
            warped = warp(bright_frame) 
            hsv_frame = hsv_filter(warped, 1, 15, 5) # (frame, wrap?, wrap value, kernel size)
           
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            pipe_msg = CvBridge().cv2_to_imgmsg(hsv_frame, encoding="8UC1") 
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            pipe_pub.publish(pipe_msg) 
            
        rate.sleep()
