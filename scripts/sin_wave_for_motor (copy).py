#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# import geometry_msgs/Twist for control commands
from geometry_msgs.msg import Twist
import math
import time
from std_msgs.msg import Float32


def bound(lower, val, upper):

	if val > upper:
		return upper
	elif val < lower:
		return lower
	else:
		return val
		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('ball_simulator', anonymous = True)
	# declare a publisher to publish in the velocity command topic
	ball_pub = rospy.Publisher('/ball_pos', Twist, queue_size = 10)
	# publishing at the rate of the camera
	loop_rate = rospy.Rate(60) 
	
	# declare a variable of type Twist for sending simulated ball positions
	ball_pos = Twist()
	
	r0 = 0
	r1 = 50
	dt = 1/60
	
	toggle_flag = 0
	while not rospy.is_shutdown():
		
		if (r0 >= 359) or (r0 <= 1):
			r1 = -r1 # switch direction
			
		r0 = bound(0, r0 + r1 * dt, 360)
		
		ball_pos.linear.y = r0
		ball_pos.angular.y = r1
	
		if ball_pos != None:
			ball_pub.publish(ball_pos)
		
		loop_rate.sleep()
		
		
