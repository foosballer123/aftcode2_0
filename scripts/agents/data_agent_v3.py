#!/usr/bin/env python3
"""
data_agent_v3.py

Records foosball experiment data to a CSV file.

Data flow changes from prior version:
    - /omega_d is now split into /omega_d_y (linear commands) and /omega_d_x (angular commands).
    - Player positions come from /rod{N}_player_positions (Float64MultiArray).
        * Linear rods (odd motor numbers: 1, 3, 5) publish 3 y-positions in meters.
        * Angular rods (even motor numbers: 2, 4, 6) publish 3 angles in radians.
    - Ball position on /ball_pos is now in meters (pos) and meters/sec (vel).

Usage examples:
    # Record Y-solver test only (motor 1 -> rod 1 linear)
    rosrun <pkg> data_agent_v3.py --solver y --motors 1 --file_name y_test.csv

    # Record X-solver test only (motor 2 -> rod 1 angular)
    rosrun <pkg> data_agent_v3.py --solver x --motors 2 --file_name x_test.csv

    # Record both solvers together for rod 1 (motors 1 and 2)
    rosrun <pkg> data_agent_v3.py --solver xy --motors 1 2 --file_name xy_test.csv

Ball is recorded by default; pass --no_ball to skip it.

After 10 consecutive duplicate samples the script terminates (same stall-detection
behavior as the original).
"""

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import argparse
import csv


# =========================
# Globals for callbacks
# =========================
# Ball: [x, y, vx, vy] in meters / meters-per-second
ball_state = [0.0, 0.0, 0.0, 0.0]
ball_received = False

# omega_d_y is a Twist with linear.{x,y,z} carrying commands for linear rods 1, 3, 5.
# omega_d_x is a Twist with angular.{x,y,z} carrying commands for angular rods 2, 4, 6.
omega_d_y = [0.0, 0.0, 0.0]  # indexed by rod number (1,2,3)
omega_d_x = [0.0, 0.0, 0.0]
omega_y_received = False
omega_x_received = False

# Player positions keyed by motor number.
# Linear motors:   list of 3 y-positions  (meters)
# Angular motors:  list of 3 theta values (radians) — typically identical across the 3 players on one rod
player_positions = {}
player_received = {}


# =========================
# Callback factories
# =========================
def ball_cb(msg):
    global ball_state, ball_received
    ball_state = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
    if not ball_received:
        ball_received = True
        print(f"[data_agent] Ball received: pos=({ball_state[0]:.3f}, {ball_state[1]:.3f}) m")


def omega_y_cb(msg):
    global omega_d_y, omega_y_received
    # linear.x -> rod 1, linear.y -> rod 2 (unused for linear), linear.z -> rod 3
    # The convention in velocity_ctrl.py is: motor 1 uses linear.x, motor 3 uses linear.y, motor 5 uses linear.z.
    # We store by "logical rod index" 1..3 where 1->linear.x, 2->linear.y, 3->linear.z.
    omega_d_y = [msg.linear.x, msg.linear.y, msg.linear.z]
    if not omega_y_received:
        omega_y_received = True
        print(f"[data_agent] omega_d_y received: {omega_d_y}")


def omega_x_cb(msg):
    global omega_d_x, omega_x_received
    # angular.x -> motor 2, angular.y -> motor 4, angular.z -> motor 6
    omega_d_x = [msg.angular.x, msg.angular.y, msg.angular.z]
    if not omega_x_received:
        omega_x_received = True
        print(f"[data_agent] omega_d_x received: {omega_d_x}")


def make_player_cb(motor_num):
    def cb(msg):
        player_positions[motor_num] = list(msg.data)
        if not player_received.get(motor_num, False):
            player_received[motor_num] = True
            print(f"[data_agent] rod{motor_num} player positions received: {player_positions[motor_num]}")
    return cb


# =========================
# Main
# =========================
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Records foosball ROS topics to a CSV file.")
    parser.add_argument("--file_name", required=True, type=str, help="Output CSV filename.")
    parser.add_argument("--sample_time", type=int, default=60, help="Sample rate in Hz (default: 60).")
    parser.add_argument("--solver", choices=["x", "y", "xy"], required=True,
                        help="Which solver(s) are under test. 'y'=linear, 'x'=angular, 'xy'=both.")
    parser.add_argument("--motors", type=int, nargs="+", required=True,
                        help="Motor numbers to record player positions from (e.g. --motors 1 2).")
    parser.add_argument("--no_ball", action="store_true", help="Skip recording /ball_pos.")

    args = parser.parse_args()

    rospy.init_node('data_agent_v3', anonymous=True)

    # -------- Subscribers --------
    if not args.no_ball:
        rospy.Subscriber('/ball_pos', Twist, ball_cb)

    if args.solver in ("y", "xy"):
        rospy.Subscriber('/omega_d_y', Twist, omega_y_cb)
    if args.solver in ("x", "xy"):
        rospy.Subscriber('/omega_d_x', Twist, omega_x_cb)

    for m in args.motors:
        topic = f'/rod{m}_player_positions'
        player_received[m] = False
        rospy.Subscriber(topic, Float64MultiArray, make_player_cb(m))
        print(f"[data_agent] Subscribed to {topic}")

    rate = rospy.Rate(args.sample_time)

    # -------- Readiness condition --------
    def all_ready():
        if not args.no_ball and not ball_received:
            return False
        if args.solver in ("y", "xy") and not omega_y_received:
            return False
        if args.solver in ("x", "xy") and not omega_x_received:
            return False
        for m in args.motors:
            if not player_received.get(m, False):
                return False
        return True

    # -------- Record loop --------
    header_written = False
    repeat = 0
    prev_data = []
    t_start = None

    print(f"[data_agent] Waiting for all topics...")

    with open(args.file_name, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        while not rospy.is_shutdown():
            if not all_ready():
                rate.sleep()
                continue

            if t_start is None:
                t_start = rospy.Time.now()
                print(f"[data_agent] All topics ready. Recording to {args.file_name}")

            t_e = (rospy.Time.now() - t_start).to_sec()

            # ---- Build row ----
            row = []
            header = []

            if not args.no_ball:
                row.extend(ball_state)  # x, y, vx, vy (m, m, m/s, m/s)
                header.extend(['ball_x', 'ball_y', 'ball_vx', 'ball_vy'])

            # Player positions — one column per player per rod
            for m in sorted(args.motors):
                pos = player_positions.get(m, [0.0, 0.0, 0.0])
                if (m % 2) == 1:
                    # Linear rod -> y-position of each of 3 players in meters
                    for i, p in enumerate(pos[:3], start=1):
                        row.append(p)
                        header.append(f'rod{m}_player{i}_y')
                else:
                    # Angular rod -> theta (rad). The rod is rigid, so all 3 entries are the same,
                    # but we write all 3 for symmetry with the publisher.
                    for i, p in enumerate(pos[:3], start=1):
                        row.append(p)
                        header.append(f'rod{m}_player{i}_theta')

            # Commanded velocities (rad/s from the solvers)
            if args.solver in ("y", "xy"):
                # Match the channel to the motor: motor 1 -> idx 0, motor 3 -> idx 1, motor 5 -> idx 2
                for m in sorted(args.motors):
                    if (m % 2) == 1:
                        idx = (m - 1) // 2  # 1->0, 3->1, 5->2
                        row.append(omega_d_y[idx])
                        header.append(f'omega_d_y_rod{m}')

            if args.solver in ("x", "xy"):
                for m in sorted(args.motors):
                    if (m % 2) == 0:
                        idx = (m - 2) // 2  # 2->0, 4->1, 6->2
                        row.append(omega_d_x[idx])
                        header.append(f'omega_d_x_rod{m}')

            row.append(t_e)
            header.append('time')

            if not header_written:
                writer.writerow(header)
                print("[data_agent] Wrote header:", header)
                header_written = True

            # Duplicate detection (excluding timestamp)
            if row[:-1] == prev_data:
                repeat += 1
            else:
                repeat = 0

            writer.writerow(row)
            prev_data = row[:-1]

            if repeat == 10:
                print("[data_agent] Data stalled. Stopping.")
                break

            rate.sleep()

    print(f"[data_agent] Wrote {args.file_name}.")
