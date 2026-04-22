#!/usr/bin/env python3
import rospy
import math
import yaml
from geometry_msgs.msg import Twist

class BallFilter:
    def __init__(self):
        rospy.init_node('ball_filter')

        # 1. Load scale from your config.yaml
        # (Assuming the file is in your current workspace)
        self.v_max = 15.0  # meters/second. A very fast foosball shot.
        self.dt = 1.0 / 60.0
        self.gate_dist = self.v_max * self.dt # ~0.1 meters per frame

        # 2. Tuning (Minimal)
        # alpha = 1.0 is raw data, 0.1 is very heavy smoothing.
        self.pos_alpha = 0.7  # High = Low lag (important for strikers)
        self.vel_alpha = 0.3  # Low = Smooth motion (important for solver stability)

        self.last_x = None
        self.last_y = None
        self.filt_vx = 0.0
        self.filt_vy = 0.0

        self.sub = rospy.Subscriber('/ball_pos_raw', Twist, self.callback)
        self.pub = rospy.Publisher('/ball_pos', Twist, queue_size=1)

    def callback(self, msg):
        raw_x = msg.linear.x
        raw_y = msg.linear.y

        # Initialize
        if self.last_x is None:
            self.last_x, self.last_y = raw_x, raw_y
            return

        # 3. GATING: Protect against "Teleporting" blips
        # If the ball moved > 10cm in 0.016s, it's a sensor error.
        dist = math.sqrt((raw_x - self.last_x)**2 + (raw_y - self.last_y)**2)
        if dist > self.gate_dist:
            # We ignore the bad reading and use the last known good position
            raw_x, raw_y = self.last_x, self.last_y

        # 4. SMOOTHING (Low Pass Filter)
        smooth_x = (self.pos_alpha * raw_x) + (1 - self.pos_alpha) * self.last_x
        smooth_y = (self.pos_alpha * raw_y) + (1 - self.pos_alpha) * self.last_y

        # 5. VELOCITY: Calculate from smoothed positions
        # This prevents the solver from seeing "infinite" spikes
        inst_vx = (smooth_x - self.last_x) / self.dt
        inst_vy = (smooth_y - self.last_y) / self.dt
        
        self.filt_vx = (self.vel_alpha * inst_vx) + (1 - self.vel_alpha) * self.filt_vx
        self.filt_vy = (self.vel_alpha * inst_vy) + (1 - self.vel_alpha) * self.filt_vy

        # 6. PUBLISH TO SOLVERS
        out = Twist()
        out.linear.x, out.linear.y = smooth_x, smooth_y
        out.angular.x, out.angular.y = self.filt_vx, self.filt_vy
        self.pub.publish(out)

        self.last_x, self.last_y = smooth_x, smooth_y

if __name__ == '__main__':
    try:
        BallFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
