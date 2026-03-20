#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

img_received = False

rgb_img = np.zeros((360, 640, 3), dtype = "uint8")

# get the image message
def get_image(ros_img):
    global rgb_img
    global img_received
    # convert to opencv image
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
    # raise flag
    img_received = True
    
if __name__ == '__main__':

    rospy.init_node('/basler_camera', anonymous = True)
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image)
    
    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
     
        if img_received:
            frame = rgb_img
