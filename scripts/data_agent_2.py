#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import argparse 
import csv
import math

# Defining data variables and flags
player_pos = 0
ball_pos = [[0,0],[0,0]] # [position (x,y), velocity (x,y)]
omega_d = 0
pos_received = False
ball_received = False
cmd_received = False

def get_pos(msg): 
    global player_pos, pos_received
    player_pos = msg.data 
    if not pos_received:
        pos_received = True
        print(f"Pos {player_pos} received!")
        
def get_ball(msg): 
    global ball_pos, ball_received
    ball_pos = [[msg.linear.x, msg.linear.y], [msg.angular.x, msg.angular.y]]
    if not ball_received:
        ball_received = True
        print(f"Ball Pos {ball_pos} received!")	

def get_cmd(msg): 
    global omega_d, cmd_received
    pps = 120 / 525 
    rpp = math.pow(pps,-1) * (2*math.pi / 400) 
    omega_d = msg.linear.x / rpp 
    if not cmd_received:
        cmd_received = True
        print(f"Cmd {omega_d} received!")
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Records specific ROS topics to a CSV file."
    )
    parser.add_argument("--sample_time", type=int, default=60)
    parser.add_argument("--file_name", required=True, type=str)
    
    # New arguments for topics
    parser.add_argument("--pos_topic", type=str, help="Topic for motor position (Float32)")
    parser.add_argument("--ball_topic", type=str, help="Topic for ball position (Twist)")
    parser.add_argument("--cmd_topic", type=str, help="Topic for omega_d (Twist)")
    
    args = parser.parse_args()
    
    rospy.init_node('data_agent', anonymous=True)
    
    # Only subscribe if the topic was provided
    if args.pos_topic:
        rospy.Subscriber(args.pos_topic, Float32, get_pos)
    if args.ball_topic:
        rospy.Subscriber(args.ball_topic, Twist, get_ball)
    if args.cmd_topic:
        rospy.Subscriber(args.cmd_topic, Twist, get_cmd)
    
    rate = rospy.Rate(args.sample_time)
    
    header_written = False
    repeat = 0
    p_data = []
    t_s = rospy.Time.now()

    with open(args.file_name, 'w', newline='') as csvfile:
        spamwriter = csv.writer(csvfile, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
        
        while not rospy.is_shutdown():
            # Check if all REQUESTED topics have received at least one message
            ready = (not args.pos_topic or pos_received) and \
                    (not args.ball_topic or ball_received) and \
                    (not args.cmd_topic or cmd_received)

            if ready:
                t_e = (rospy.Time.now() - t_s).to_sec()
                
                # Build dynamic data row and header
                current_data = []
                current_header = []
                
                if args.ball_topic:
                    current_data.extend([ball_pos[0][0], ball_pos[0][1], ball_pos[1][0], ball_pos[1][1]])
                    current_header.extend(['ball_x','ball_y','ball_vx','ball_vy'])
                if args.pos_topic:
                    current_data.append(player_pos)
                    current_header.append('player_y1')
                if args.cmd_topic:
                    current_data.append(omega_d)
                    current_header.append('omega_d')
                
                current_data.append(t_e)
                current_header.append('time')

                if not header_written:
                    spamwriter.writerow(current_header)
                    print("Wrote Header:", current_header)
                    header_written = True

                # Duplicate detection (excluding timestamp)
                if current_data[:-1] == p_data:
                    repeat += 1
                else:
                    repeat = 0
                
                spamwriter.writerow(current_data)
                p_data = current_data[:-1]
                
                if repeat == 10:
                    print("Data stalled. Stopping...")
                    break
                    
            rate.sleep()
