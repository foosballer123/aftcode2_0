#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import argparse 
import csv
import subprocess

def terminate_process_and_children(p):
	import psutil
	process = psutil.Process(p.pid)
	for sub_process in process.children(recursive=True):
		sub_process.send_signal(signal.SIGINT)
	p.wait()  # we wait for children to terminate

def main():
	dir_save_bagfile = '~/catkin_ws/src/aftcode2_0/rosbags/'
	rosbag_process = subprocess.Popen('rosbag play experiment_5_p45_hv_mode1_test3_1771903734.bag', stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
	terminate_process_and_children(rosbag_process)

if __name__ == '__main__':
	main()
