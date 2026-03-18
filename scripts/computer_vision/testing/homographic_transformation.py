import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 
import normalizing as norm

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
def warp(image):

    img = image.copy()
	
    # corners of the field in the original image
    pts1 = np.float32([[23,7],[619,12],[19,351],[618,354]])
    # 618 354     19 351
    # mapping to a new image 
    pts2 = np.float32([[0,0],[640,0],[0,360],[640,360]])

    # find the corresponding transformation
    M = cv2.getPerspectiveTransform(pts1,pts2)
    
    # use the transformations to warp the initial image to a new plane/image 
    dst = cv2.warpPerspective(img,M,(640,360))


    return dst
    
if __name__ == '__main__': 
	
    rospy.init_node('homographic_transformation', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    
    warp_pub = rospy.Publisher('/warped', Image, queue_size = 1) 
    img_pub = rospy.Publisher('/marked_img', Image, queue_size = 1)
    
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img
            image = frame.copy() 
            #frame = norm.normalize(frame)
            
            warped = warp(frame) 
            
            cv2.line(warped, (0,0), (640, 360), (255, 255, 255), 2)
            cv2.line(warped, (0,360), (640, 0), (255, 255, 255), 2)
            cv2.line(warped, (0,120), (640, 120), (255, 0, 0), 3)
            cv2.line(warped, (0,240), (640, 240), (255, 0, 0), 3)
            
            cv2.line(image, (0,0), (640, 360), (255, 255, 255), 2)
            cv2.line(image, (0,360), (640, 0), (255, 255, 255), 2)
            cv2.line(image, (0,120), (640, 120), (255, 0, 0), 3)
            cv2.line(image, (0,240), (640, 240), (255, 0, 0), 3)
            
            cv2.circle(warped, (0, 0), radius=10, color=(0, 0, 255), thickness=-1) 
            cv2.circle(image, (0, 0), radius=10, color=(0, 0, 255), thickness=-1) 
            
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            warp_msg = CvBridge().cv2_to_imgmsg(warped, encoding="bgr8") 
            img_msg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8") 
            
            #img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
            warp_pub.publish(warp_msg) 
            img_pub.publish(img_msg)
            
        rate.sleep()
