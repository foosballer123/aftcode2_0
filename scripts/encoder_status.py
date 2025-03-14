#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String

# Define GPIO pins for sensors
SENSOR_1 = 5
SENSOR_2 = 6
SENSOR_3 = 26
SENSOR_4 = 7
SENSOR_5 = 12
SENSOR_6 = 4

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(SENSOR_2, GPIO.IN)
GPIO.setup(SENSOR_3, GPIO.IN)
GPIO.setup(SENSOR_4, GPIO.IN)
GPIO.setup(SENSOR_5, GPIO.IN)
GPIO.setup(SENSOR_6, GPIO.IN)

def read_sensors():
	sensor_data = {
	"SENSOR_1": GPIO.input(SENSOR_1),
	"SENSOR_2": GPIO.input(SENSOR_2),
	"SENSOR_3": GPIO.input(SENSOR_3),
	"SENSOR_4": GPIO.input(SENSOR_4),
	"SENSOR_5": GPIO.input(SENSOR_5),
	"SENSOR_6": GPIO.input(SENSOR_6),
	}
	return sensor_data

if __name__ == "__main__":
	#init ros node
	rospy.init_node('sensor_publisher', anonymous=True)
	
	#init ros publisher
	pub = rospy.Publisher('sensor_data', String, queue_size=10)	
	#set publishing rate 1/10 second
	rate = rospy.Rate(10)
	
	rospy.loginfo("Starting sensor data stream...")
		
	try:
		while not rospy.is_shutdown():
			#read data 
			sensor_data = read_sensors()
			
			#convert to string for publishing
			sensor_data_str = str(sensor_data)
			
			#publish the data
			pub.publish(sensor_data_str)
			rate.sleep()
			
	except rospy.ROSInterruptExecption:
		rospy.loginfo("Shutting down sensor data publisher...")
	finally:
		#clean the pins
		GPIO.cleanup()
		

