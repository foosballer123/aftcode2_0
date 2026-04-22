#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist

class KalmanFilter2D:
    def __init__(self, dt, r_noise, q_noise):
        self.dt = dt
        # State: [x, vx, y, vy]
        self.x = np.zeros((4, 1))
        
        # State Transition Matrix (Constant Velocity)
        self.F = np.array([[1, dt, 0, 0],
                           [0, 1,  0, 0],
                           [0, 0,  1, dt],
                           [0, 0,  0, 1]])
        
        # Measurement Matrix (We only measure position x and y)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
        
        self.P = np.eye(4) * 1.0  # Initial covariance
        self.R = np.eye(2) * r_noise  # Measurement noise
        self.Q = np.eye(4) * q_noise  # Process noise

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        # z = [measured_x, measured_y]
        y = z - np.dot(self.H, self.x)
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot((np.eye(4) - np.dot(K, self.H)), self.P)
        return self.x

class BallFilterNode:
    def __init__(self):
        rospy.init_node('ball_filter_node')
        
        # Parameters from your config.yaml
        #fps = rospy.get_param('/pylon_camera_node/frame_rate', 60)
        fps = 60
        self.dt = 1.0 / fps
        
        # Tuning: 
        # R (Measurement Noise): Higher values trust the model more than camera jitter.
        # Q (Process Noise): Higher values allow the filter to react faster to bounces.
        self.kf = KalmanFilter2D(dt=self.dt, r_noise=0.010, q_noise=0.10)
        
        self.sub = rospy.Subscriber('/ball_pos_raw', Twist, self.callback)
        self.pub = rospy.Publisher('/ball_pos', Twist, queue_size=10)
        self.filtered_msg = Twist()

    def callback(self, msg):
        # 1. Prediction step
        self.kf.predict()
        
        # 2. Measurement update
        measured_z = np.array([[msg.linear.x], [msg.linear.y]])
        state = self.kf.update(measured_z)
        
        # 3. Pack filtered data back into Twist format for Solvers
        self.filtered_msg.linear.x = state[0, 0] # Filtered Position X
        self.filtered_msg.linear.y = state[2, 0] # Filtered Position Y
        self.filtered_msg.angular.x = state[1, 0] # Filtered Velocity X
        self.filtered_msg.angular.y = state[3, 0] # Filtered Velocity Y
        
        self.pub.publish(self.filtered_msg)

if __name__ == '__main__':
    node = BallFilterNode()
    rospy.spin()
