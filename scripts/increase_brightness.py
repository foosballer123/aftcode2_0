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
def increase_brightness(image, num):

    img = image #.copy()
    img = np.where(img + num < 255, img + num, 255)
    img = np.array(img, dtype=np.uint8)
    
    """
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
    """
    
    return img
    
def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img
    
if __name__ == '__main__': 
	
    rospy.init_node('increase_brightness', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    bright_pub = rospy.Publisher('/increase_brightness', Image, queue_size = 1) 
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
          
            bright = increase_brightness(frame, 30) 
          
            
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            bright_msg = CvBridge().cv2_to_imgmsg(bright, encoding="bgr8")
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            bright_pub.publish(bright_msg) 
            
        rate.sleep()
