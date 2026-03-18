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

#cap = cv2.VideoCapture("Videos/17output.mp4") 

#frame_rate = cap.get(cv2.CAP_PROP_FPS) 

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360) 

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 
  
def mask_image(frame): 

    h, w = frame.shape[:2] 
   
    # create a mask that will be used to isolate the players 
    mask = np.zeros((h, w), dtype=np.uint8) 
   	
    mask_table = mask.copy() 
       
    # mask to remove the borders of the table (needs to be adjusted with camera frame) 
    x1, y1, x2, y2 = (0, 12, 615, 355) 
    cv2.rectangle(mask_table, (x1, y1), (x2, y2), color=255, thickness=-1) 
    frame = cv2.bitwise_and(frame, frame, mask=mask_table) 
   
    # isolate the rods 
    rod_mask = pt_m.rod_mask(frame) 
    red_points, binary_mask = pt_m.track_red(rod_mask) 

    print(red_points) 

    mask_search = mask.copy() 
    img_search = frame.copy() 
   
    # the parameters need to be tuned based on the camera position 
    # the rods are at fixed positions, so we can use the x-coordinates to isolate the players 
    x_r_1 = 170 
    x_r_2 = 380 
    x_r_3 = 590 

    # create rectangles around each player to use to search for furthur keypoints 
    for i in red_points: 

        if abs(i[0] - x_r_1) < 15: 
            print(i) 
            x1, y1, x2, y2 = (x_r_1 - 70, i[1] - 15, x_r_1 + 70, i[1] + 15) 
            cv2.rectangle(mask_search, (x1, y1), (x2, y2), color=255, thickness=-1) 
        elif abs(i[0] - x_r_2) < 15: 
            print(i) 
            x1, y1, x2, y2 = (x_r_2 - 70, i[1] - 15, x_r_2 + 70, i[1] + 15) 
            cv2.rectangle(mask_search, (x1, y1), (x2, y2), color=255, thickness=-1) 
        elif abs(i[0] - x_r_3) < 15: 
            print(i) 
            x1, y1, x2, y2 = (x_r_3 - 70, i[1] - 15, x_r_3 + 70, i[1] + 15) 
            cv2.rectangle(mask_search, (x1, y1), (x2, y2), color=255, thickness=-1) 

    masked_search = cv2.bitwise_and(img_search, img_search, mask=mask_search) 

    red_points_ext, binary_mask_ext = pt_m.track_red(masked_search) 
    
    # Why am I drawing two different sets of point over the same image?
    pt.draw_points(masked_search, red_points, (255, 0, 0)) 
    pt.draw_points(masked_search, red_points_ext, (255, 0, 0)) 

    print(red_points_ext) 

    distances = [] 

    # for every coordinate in red_points_ext 
    for i in range(len(red_points_ext)): 
        # compare it to every other coordinate 
        for j in range(len(red_points_ext)): 
            # make sure it is not the exact same coordinate 
            if red_points_ext[j] != red_points_ext[i]: 
                # and check if the y-coordinate is within 10 pixels of each other 
                if abs(red_points_ext[i][1] - red_points_ext[j][1]) <= 10: 
                    # then check if the x-coordinate is within 75 pixels of each other 
                    if abs(red_points_ext[i][0] - red_points_ext[j][0]) <= 75: 
                        # then calculate the distance between the two points 
                        dx = abs(red_points_ext[i][0] - red_points_ext[j][0]) 
						
						# and draw a line between those two points
                        cv2.line(masked_search, (red_points_ext[i][0], 
red_points_ext[i][1]), (red_points_ext[j][0], 
red_points_ext[j][1]),(255,0,0),5) 
                        distances.append(dx) 
                        
                    # checking if point is along one of the player rods 
                    if (abs(red_points_ext[i][0] - x_r_1) < 15) or (abs(red_points_ext[i][0] - x_r_2) < 15) or (abs(red_points_ext[i][0] - x_r_3) < 15): 
                        
                        # finding out which player rod the point is a part of 
                        if (abs(red_points_ext[i][0] - x_r_1) < 15): 
                            x_r = x_r_1 
                        elif (abs(red_points_ext[i][0] - x_r_2) < 15): 
                            x_r = x_r_2 
                        elif (abs(red_points_ext[i][0] - x_r_3) < 15): 
                            x_r = x_r_3 
                        
                        # finding the point on the same y-plane 
                        for k in range(len(red_points_ext)): 
                        
                            if (abs(red_points_ext[i][1] - red_points_ext[k][1]) <= 10) and (abs(red_points_ext[k][0] - x_r) < 80): 

                                cv2.line(masked_search, (red_points_ext[i][0], red_points_ext[i][1]), (red_points_ext[j][0], red_points_ext[k][0]),(255,0,0),5) 
                             
    # the mean of the distances can be used to calculate the angle of the players along the same rod 
    # note: it doesn't matter if every key point is detected because we are taking the average! 
    # note: if the list is empty, it probably means that the player is in base position 

    # note: when taking multiple rods into account, it would be wise to clump groups of similar points together before taking the average 
     
    print(distances) 

    # mask_final = mask.copy() 
    # 
    # # create a rectangle around the player (foot - torso) 
    # a1, b1, a2, b2 = (x_r - 65, red_points[1][1] - 7, x_r, red_points[1][1] + 7) 
    # 
    # # create a rectangle around the player (torso - heat) 
    # c1, d1, c2, d2 = (x_r, red_points[1][1] - 7, x_r + 10, red_points[1][1] + 7) 
    # 
    # cv2.rectangle(mask_final, (a1, b1), (a2, b2), color=255, thickness=-1) 
    # cv2.rectangle(mask_final, (c1, d1), (c2, d2), color=255, thickness=-1) 
    # 
    # masked_final = cv2.bitwise_and(img, img, mask=mask_final) 
   
    #cv2.imshow('search mask', masked_search) 
    #cv2.imshow('final mask', masked_final) 

    return masked_search, rod_mask, binary_mask, binary_mask_ext 

if __name__ == '__main__': 

    rospy.init_node('masking_pipeline', anonymous = True) 
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    img_pub = rospy.Publisher('/masked_players', Image, queue_size = 1) 
    img_pub_2 = rospy.Publisher('/rod_mask', Image, queue_size = 1) 
    img_pub_3 = rospy.Publisher('/binary_mask', Image, queue_size = 1) 
    img_pub_4 = rospy.Publisher('/binary_mask_ext', Image, queue_size = 1) 
     
    rate = rospy.Rate(60) 
   
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img 
          
            out_players, out_rod, binary_mask, binary_mask_ext = mask_image(frame) 
             
            # Note: The original 'rod_mask' is a grayscale image (single channel). 
            # If the publisher expects a color image (like the others), converting it to BGR8 
            # will treat the single channel as all three channels (creating a visible grayscale image). 
            img_msg = CvBridge().cv2_to_imgmsg(out_players, encoding="bgr8") 
            img_msg_2 = CvBridge().cv2_to_imgmsg(out_rod, encoding="bgr8") 
            img_msg_3 = CvBridge().cv2_to_imgmsg(binary_mask, encoding="8UC1") 
            img_msg_4 = CvBridge().cv2_to_imgmsg(binary_mask_ext, encoding="8UC1") 
             
            img_pub.publish(img_msg) 
            img_pub_2.publish(img_msg_2) 
            img_pub_3.publish(img_msg_3) 
            img_pub_4.publish(img_msg_4) 
             
        rate.sleep()
