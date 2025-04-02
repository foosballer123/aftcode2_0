#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def command_sender():
    rospy.init_node('motor_command_publisher', anonymous=True)
    pub = rospy.Publisher('motor_commands', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        command = input("Enter command (step/rotate/stop): ").strip().lower()
        if command in {"step", "rotate", "stop"}:
            rospy.loginfo(f"Publishing command: {command}")
            pub.publish(command)
            if command == "stop":
                break
        else:
            rospy.logwarn("Invalid command. Please enter 'step', 'rotate', or 'stop'.")
        rate.sleep()

if __name__ == '__main__':
    try:
        command_sender()
    except rospy.ROSInterruptException:
        pass

