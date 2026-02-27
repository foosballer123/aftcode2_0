import cv2 
import numpy as np 
import b_player_tracking_with_masking as pt_m 
import player_tracking as pt 
import normalizing as n
import homographic_transformation as ht
import increase_brightness as ib
import math

import rospy 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

# Remember: 
# X positions of players are fixed (because they are fixed to a rod) 
# Y positions of players are variable 

img_received = False 

rgb_img = np.zeros((360, 640, 3), dtype = "uint8") 

motor1_pos = 0
ball_pos = Twist()
horizon = Twist()
horizon_x, horizon_y = 0, 0

tvp = 0
tvp_received = False
ball_received = False
player_received = False
angular_received = False
horizon_received = False

dist = 112 # distance between players along the same rod
rod = 40 # position of the rod in the x-direction
x_foot = 0

pps = 120 / 525  # pixels per step (was 330/525 ... testing with 120/525)
rpp = math.pow(pps,-1) * (2*math.pi / 400)	# radians per pixel (was 0.025 ... testing calculation)
ppr = math.pow(rpp,-1)	# pixels per radian (was 40 ... testing calculation)

# variables for counting the frames that the solver spends approaching and following the ball
# tuples for: (closely following, kind of following, barely following)
approaching = [0,0,0]
following = [0,0,0]
metrics = Twist()

# get the image message 
def get_image(ros_img): 
    global rgb_img 
    global img_received 
    # convert to opencv image 
    rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "bgr8") 
    # raise flag 
    img_received = True 
    
def player_callback(msg):
    global motor1_pos
    global pps
    global player_received
    
    motor1_pos = pps * msg.data + 3  # converting steps into pixels
    player_received = True
    
def angular_callback(msg):
	global x_foot 
	global rod
	global angular_received
	
	L = 50
	
	offset = math.pi*0.3
	rad = msg.data*((2*math.pi)/400) + offset # steps * rad/step = rad
	x_foot = int(rod + L*math.sin(-rad))
	angular_received = True
	
def get_horizon(msg):
    global horizon
    global horizon_x
    global horizon_y
    global horizon_received
    
    horizon = msg
    horizon_x, horizon_y = horizon.linear.x, horizon.linear.y
    horizon_received = True
    
def ball_callback(msg):
 
    global ball_pos
    global ball_received
    ball_pos = msg  
    ball_received = True
    
def get_tvp(msg): 
	global tvp 
	global tvp_received
	tvp = msg.data
	tvp_received = True   
	 
def draw_box(image, points = False):

    pts = ((25,7),(616,12),(23,351),(618,352)) # approximate corners of field in image frame
    cv2.rectangle(image, pts[0], pts[-1], (120, 255, 0), thickness= 1, lineType=cv2.LINE_8) 
    
    if points:
        cv2.circle(image, pts[0], radius=3, color=(255, 255, 255), thickness=-1)
        cv2.circle(image, pts[1], radius=3, color=(255, 255, 255), thickness=-1)
        cv2.circle(image, pts[2], radius=3, color=(255, 255, 255), thickness=-1)
        cv2.circle(image, pts[3], radius=3, color=(255, 255, 255), thickness=-1)
        
    return image
    
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
    
def draw_players(image, extended = False):
    
    global motor1_pos
    global dist
    global rod
    global player_received
    global x_foot
    global angular_received
    
    if player_received:
        cv2.circle(image, (rod, 0), radius=3, color=(0, 0, 255), thickness=-1)
        cv2.circle(image, (rod, int(motor1_pos)), radius=3, color=(255, 255, 255), thickness=-1)
        cv2.circle(image, (rod, int(motor1_pos)+dist), radius=3, color=(255, 255, 255), thickness=-1)
        cv2.circle(image, (rod, int(motor1_pos)+2*dist), radius=3, color=(255, 255, 255), thickness=-1)
        if extended:
            cv2.line(image, (rod, int(motor1_pos)), (640, int(motor1_pos)), (100, 100, 255), 1)
            cv2.line(image, (rod, int(motor1_pos)+dist), (640, int(motor1_pos)+dist), (100, 100, 255), 1)
            cv2.line(image, (rod, int(motor1_pos)+2*dist), (640, int(motor1_pos)+2*dist), (100, 100, 255), 1)
    
    if angular_received:
        cv2.circle(image, (x_foot, int(motor1_pos)), radius=3, color=(150, 150, 255), thickness=-1)
        cv2.circle(image, (x_foot, int(motor1_pos)+dist), radius=3, color=(150, 150, 255), thickness=-1)
        cv2.circle(image, (x_foot, int(motor1_pos)+2*dist), radius=3, color=(150, 150, 255), thickness=-1)
        
    return image

# metric for calculating the % time that the ball spends spends within a given range of the players foot
def calculate_metrics():

    global approaching
    global following
    global ball_pos
    global ball_received
    global player_received
    global motor1_pos
    global dist
   
    a,b,c = 3,7,15 # (a) closely following, (b) kind of following, (c) barely following
    
    if ball_received and player_received:
    
        player1 = motor1_pos # player furthest from hardware
        player2 = motor1_pos+dist
        player3 = motor1_pos+2*dist
        
        y_ball = ball_pos.linear.y
        
        if ((abs(player1-y_ball) <= a) or (abs(player2-y_ball) <= a) or (abs(player3-y_ball) <= a)):
            following[0] += 1
        else:
            approaching[0] += 1
        if ((abs(player1-y_ball) <= b) or (abs(player2-y_ball) <= b) or (abs(player3-y_ball) <= b)):
            following[1] += 1
        else:
            approaching[1] += 1
        if ((abs(player1-y_ball) <= c) or (abs(player2-y_ball) <= c) or (abs(player3-y_ball) <= c)):
            following[2] += 1
        else:
            approaching[2] += 1
            
        metrics.linear.x = following[0]/(following[0]+approaching[0])
        metrics.linear.y = following[1]/(following[1]+approaching[1])
        metrics.linear.z = following[2]/(following[2]+approaching[2])
        
def draw_zones(image, cross = True):
    
    if cross == True:
        cv2.line(image, (0,0), (640, 360), (255, 255, 255), 2)
        cv2.line(image, (0,360), (640, 0), (255, 255, 255), 2)
    cv2.line(image, (0,120), (640, 120), (255, 120, 120), 2)
    cv2.line(image, (0,240), (640, 240), (255, 120, 120), 2) 

    return image
    
def draw_coords(image, origin = True):
    
    global motor1_pos
    global dist
    global rod
    global player_received
    global ball_received
    global tvp_received
    global ball_pos
    global tvp
    
    if origin == True:           
        cv2.putText(image, "(0,0)", (0,shift+10), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))      
    cv2.putText(image, "0", (450,0+shift), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    cv2.putText(image, "120", (450,120+shift), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    cv2.putText(image, "240", (450,240+shift), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    cv2.putText(image, "360", (450,360), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    
    if player_received:
        cv2.putText(image, str(int(motor1_pos)), (rod+50,int(motor1_pos)+5), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
        cv2.putText(image, str(int(motor1_pos)+dist), (rod+50,int(motor1_pos)+dist+5), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
        cv2.putText(image, str(int(motor1_pos)+2*dist), (rod+50,int(motor1_pos)+2*dist+5), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    if ball_received:
        cv2.putText(image, str(int(ball_pos.linear.x))+","+str(int(ball_pos.linear.y)), (int(ball_pos.linear.x)+20,int(ball_pos.linear.y)), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
    if tvp_received:
        cv2.putText(image, "("+str(int(ball_pos.linear.x))+","+str(int(tvp))+")", (int(ball_pos.linear.x),int(tvp)), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color = (0,0,0))
        
    return image
    
def draw_ball(image, offset = True, vector = True):

    global ball_pos
    global tvp_received
    global ball_received
    global horizon_x
    global horizon_y
    global horizon_received
    
    if offset:
        if tvp_received:
            cv2.circle(image, (int(ball_pos.linear.x), int(tvp)), radius=15, color=(255, 0, 0), thickness=-1)
        if horizon_received:
            cv2.circle(image, (int(horizon_x), int(horizon_y)), radius=15, color=(50, 50, 200), thickness=-1)
    if ball_received:
        r_x = int(ball_pos.linear.x)
        r_y = int(ball_pos.linear.y)
        cv2.circle(image, (r_x, r_y), radius=15, color=(255, 255, 255), thickness=3)
        if vector:
            v_x = int(ball_pos.angular.x)
            v_y = int(ball_pos.angular.y)
            
            i = int(round((v_x)/(abs(v_x)+0.001),0))
            k = int(round((v_y)/(abs(v_y)+0.001), 0))
            
            #norm_x = int(abs(abs(v_x) - 3)/3)
            #norm_y = int(abs(abs(v_y) - 3)/3)
            
            #print(norm_x, norm_y)
            cv2.line(image, (r_x,r_y), (r_x+i*50, r_y), (0, 255, 0), 2) # x vector
            cv2.line(image, (r_x,r_y), (r_x, r_y+k*50), (0, 0, 255), 2) # y vector
            
    return image
    
if __name__ == '__main__': 
	
    rospy.init_node('coordinate_pipeline', anonymous = True) 
    
    img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
    player_sub = rospy.Subscriber("/motor1_pos", Float32, player_callback, queue_size=10)
    player_angular_sub = rospy.Subscriber("/motor2_pos", Float32, angular_callback, queue_size=10)
    ball_sub = rospy.Subscriber("/ball_pos", Twist, ball_callback, queue_size=10)
    tvp_sub = rospy.Subscriber("/r0_tvp", Float32, get_tvp)
    horizon_sub = rospy.Subscriber("/r0_horizon", Twist, get_horizon)
    
    pipe_pub = rospy.Publisher('/coordinate_pipeline', Image, queue_size = 1) 
    met_pub = rospy.Publisher('/metrics', Twist, queue_size = 1)
    
    #rate = rospy.Rate(60) 
    shift = 5
    
    while not rospy.is_shutdown(): 
   
        if img_received: 
            frame = rgb_img
            frame = warp(frame) 
            #frame = draw_box(frame, points = True)
            frame = draw_zones(frame)
            frame = draw_players(frame, extended = True)
            frame = draw_coords(frame)
            frame = draw_ball(frame)
            
            calculate_metrics()
            
            pipe_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8") # grayscale: 8UC1
            
            pipe_pub.publish(pipe_msg) 
            met_pub.publish(metrics)
            
        #rate.sleep()
