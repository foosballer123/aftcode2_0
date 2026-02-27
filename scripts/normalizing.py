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
def normalize(image):

    img = image
    
    B, G, R = cv2.split(img)
    
    R = (((R-R.min())/(R.max()-R.min()))*255.0).astype(np.uint8)
    G = (((G-G.min())/(G.max()-G.min()))*255.0).astype(np.uint8)
    B = (((B-B.min())/(B.max()-B.min()))*255.0).astype(np.uint8)
    
    img = cv2.merge([B, G, R])
    
    return img
    
if __name__ == '__main__': 
	
    rospy.init_node('normalizer', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    norm_pub = rospy.Publisher('/normalized_image', Image, queue_size = 1) 
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
          
            normal = normalize(frame) 
         
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            norm_msg = CvBridge().cv2_to_imgmsg(normal, encoding="bgr8") 
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            norm_pub.publish(norm_msg)
            
        rate.sleep()
