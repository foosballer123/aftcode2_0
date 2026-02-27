#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

enc = 0
omega_d = Twist()

# get the encoder message 
def get_encoder(msg): 
	global enc
	enc = msg.data

# initialize the node
rospy.init_node('init_agent', anonymous = True)
enc_sub = rospy.Subscriber("/motor1_right_encoder", Float32, get_encoder)
cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size = 1)

while enc != 1:
	print("[Enc is "+str(enc)+"] Initializing...")
	omega_d.linear.x = 5
	cmd_pub.publish(omega_d)
print("[Enc is "+str(enc)+"] Initialized.")
