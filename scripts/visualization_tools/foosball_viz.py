#!/usr/bin/env python3
import rospy
import pygame
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import foosbot_physics as phys # Your shared logic

class FoosballVisualizer:
    def __init__(self):
        rospy.init_node('foosball_viz')
        
        # 1. Load Dimensions from config.yaml
        self.field_w = 0.590 
        self.field_h = 0.340
        self.x_rod = 0.068
        self.p_dist = 0.090
        self.l_p = 0.045
        self.ball_r = 0.015
        
        # Visual Scaling (Meters to Pixels)
        self.scale = 1000 
        self.win_w = int(self.field_w * self.scale)
        self.win_h = int(self.field_h * self.scale)

        # 2. State Variables (Populated by ROS)
        self.ball_x, self.ball_y = 0.0, 0.0
        self.rod_y = 0.0
        self.rod_theta = 0.0

        # 3. ROS Subscribers
        rospy.Subscriber("/ball_pos", Twist, self.ball_cb)
        rospy.Subscriber("/rod1_player_positions", Float64MultiArray, self.y_cb)
        rospy.Subscriber("/rod2_player_positions", Float64MultiArray, self.x_cb)

        # 4. Pygame Setup
        pygame.init()
        self.screen = pygame.display.set_mode((self.win_w, self.win_h))
        self.clock = pygame.time.Clock()

    def ball_cb(self, msg): self.ball_x, self.ball_y = msg.linear.x, msg.linear.y
    def y_cb(self, msg): self.rod_y = msg.data[0] # Takes y1
    def x_cb(self, msg): self.rod_theta = msg.data[0] # Takes theta

    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT: return

            # Clear Screen (Dark Green Field)
            self.screen.fill((30, 100, 30))

            # --- DRAW LOGIC ---
            
            # A. Draw Rod Line
            rod_px_x = int(self.x_rod * self.scale)
            pygame.draw.line(self.screen, (150, 150, 150), (rod_px_x, 0), (rod_px_x, self.win_h), 2)

            # B. Draw Players and Feet
            # Calculate positions using the functional library
            y_positions = phys.get_y_kinematics(self.rod_y, self.p_dist)
            x_foot, z_foot = phys.get_foot_dynamics(self.rod_theta, self.x_rod, self.l_p)

            for y in y_positions:
                # Draw "Torso" (Stationary on Rod X)
                pygame.draw.circle(self.screen, (0, 0, 255), (rod_px_x, int(y * self.scale)), 10)
                
                # Draw "Foot" (Moving based on Theta)
                foot_px_x = int(x_foot * self.scale)
                # Foot color dims as it goes "higher" (Z increases) to show 3D depth
                color_val = max(50, min(255, int(255 - (z_foot * 2000))))
                pygame.draw.circle(self.screen, (color_val, 0, 0), (foot_px_x, int(y * self.scale)), 8)

            # C. Draw Ball
            bx, by = int(self.ball_x * self.scale), int(self.ball_y * self.scale)
            pygame.draw.circle(self.screen, (255, 255, 255), (bx, by), int(self.ball_r * self.scale))

            pygame.display.flip()
            self.clock.tick(30) # Maintain 30 Hz

if __name__ == '__main__':
    FoosballVisualizer().run()
