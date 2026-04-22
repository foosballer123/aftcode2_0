#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class TwistCombiner:
    def __init__(self):
        rospy.init_node('twist_combiner', anonymous=True)
        
        # Subscribe to the individual solver outputs
        rospy.Subscriber('/omega_d_x', Twist, self.x_callback, queue_size=10)
        rospy.Subscriber('/omega_d_y', Twist, self.y_callback, queue_size=10)
        
        # Publish the unified command
        self.cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size=10)
        
        # The unified message we will constantly update and publish
        self.combined_cmd = Twist()
        
        # Enforce the strict 20 Hz loop rate (50ms timestep)
        self.rate = rospy.Rate(20)

    def x_callback(self, msg):
        # Cache the latest angular command from the X solver
        self.combined_cmd.angular.x = msg.angular.x

    def y_callback(self, msg):
        # Cache the latest linear command from the Y solver
        self.combined_cmd.linear.x = msg.linear.x

    def start(self):
        rospy.loginfo("Combiner node running at 20 Hz...")
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.combined_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    combiner = TwistCombiner()
    combiner.start()
