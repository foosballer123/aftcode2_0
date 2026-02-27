import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 
import normalizing as n
import homographic_transformation as ht
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
  
    
if __name__ == '__main__': 
	
    rospy.init_node('cv_pipeline', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    pipe_pub = rospy.Publisher('/pipeline', Image, queue_size = 1) 
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
            
            normalized_frame = n.normalize(frame)
            bright_frame = ib.increase_brightness(normalized_frame, 30)
            #warped = ht.warp(bright_frame) 
            
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            pipe_msg = CvBridge().cv2_to_imgmsg(bright_frame, encoding="bgr8") 
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            pipe_pub.publish(pipe_msg) 
            
        rate.sleep()
