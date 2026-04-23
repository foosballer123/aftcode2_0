#!/usr/bin/env python3

'''
Claudes version of the MPC_Y_Solver_Metric that integrates with its digital web twin interface.
This code began with the core functionality of the MPC_Y_Solver but may include changes that are not included in the original.
This is now the main functional solver that is used for Aftcode2_0.

Before the system responds to any control inputs produced by this solver make sure to run combiner_node.by in the sandbox folder.

Benjamin Simpson 4/22/2026
'''

import do_mpc
from casadi import *
from casadi.tools import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import rospy
import numpy as np
import math
import argparse
import json
import os
import datetime

class MPC_Solver:

    def __init__(self):

        rospy.init_node('mpc_y_solver', anonymous=True)

        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        rospy.Subscriber("/rod1_player_positions", Float64MultiArray, self.player_callback, queue_size=10)

        self.cmd_pub = rospy.Publisher('/omega_d_y', Twist, queue_size=10)
        # NEW: horizon publisher
        self.horizon_pub = rospy.Publisher('/y_solver/horizon', Float64MultiArray, queue_size=10)
        # NEW: recording command subscriber
        from std_msgs.msg import Bool
        rospy.Subscriber('/recording/cmd', Bool, self.recording_cmd_callback, queue_size=10)

        self.dt = rospy.get_param('/y_solver_parameters/timestep')
        self.rate = rospy.Rate(int(1/self.dt))

        self.ball_flag = False
        self.ball_pos = Twist()
        self.motor1_pos = Float64MultiArray()

        self.omega_cmd = Twist()
        self.rod_vel = 0

        self.P_D = rospy.get_param('/table_measurements/distance_between_players')
        self.Z_D = rospy.get_param('/table_measurements/distance_to_clear_zone')
        self.W_D = rospy.get_param('/table_measurements/player_distance_from_wall')
        self.zone_padding = 0.05

        self.Y = rospy.get_param('/table_measurements/field_height')
        self.Y_MAX = self.Y
        self.Y_MIN = 0

        self.steps_per_revolution = rospy.get_param('/table_measurements/steps_per_revolution')
        self.meters_per_step = self.Y / rospy.get_param('/table_measurements/steps_across_field')
        self.radians_per_meter = math.pow(self.meters_per_step, -1) * (2*math.pi / self.steps_per_revolution)
        self.meters_per_radian = math.pow(self.radians_per_meter, -1)

        # NEW: history buffer
        self.history_entries = []
        self.max_history = 100000
        self.save_dir = rospy.get_param('/y_solver_parameters/history_save_dir', '/tmp')

        # NEW: recording controlled by /recording/cmd (Bool). Off by default.
        self.record_enabled = False
        self.segment_start_t = None
        rospy.loginfo("Y solver: recording controlled by /recording/cmd topic (initially OFF)")

        rospy.on_shutdown(self.save_history_on_shutdown)

    def ball_callback(self, msg):
        if self.ball_flag == False:
            self.ball_flag = True
        self.ball_pos = msg

    def player_callback(self, msg):
        self.motor1_pos = msg.data

    def init_mpc(self):

        input_weight = rospy.get_param('/y_solver_parameters/cost_function/input_weight')
        prediction_steps = rospy.get_param('/y_solver_parameters/prediction_steps')
        input_rate_weight = rospy.get_param('/y_solver_parameters/input_rate_weight')
        position_error_weight = rospy.get_param('/y_solver_parameters/cost_function/position_error_weight')
        velocity_error_weight = rospy.get_param('/y_solver_parameters/cost_function/velocity_error_weight')
        print("Solver Parameters")
        print("-----------------------------")
        print("Input Weight:", input_weight)
        print("Position Error Weight:", position_error_weight)
        print("Velocity Error Weight:", velocity_error_weight)
        print("Prediction Horizon:", prediction_steps)
        print("Input Rate Weight:", input_rate_weight, "\n")
        v_max = rospy.get_param('/y_solver_parameters/max_input') * self.meters_per_radian

        # NEW: remember N for horizon packing
        self.N_horizon = prediction_steps

        model = do_mpc.model.Model('discrete')

        y1 = model.set_variable(var_type='_x', var_name='y1', shape=(1, 1))
        y2 = model.set_variable(var_type='_x', var_name='y2', shape=(1, 1))
        y3 = model.set_variable(var_type='_x', var_name='y3', shape=(1, 1))
        r0_y = model.set_variable(var_type='_x', var_name='r0_y', shape=(1, 1))
        r1_y = model.set_variable(var_type='_x', var_name='r1_y', shape=(1, 1))

        y_vel = model.set_variable(var_type='_u', var_name='y_vel')

        model.set_expression(expr_name="z1", expr=casadi.tanh(50*casadi.fmax(0, r0_y)))
        z1 = model.aux["z1"]
        model.set_expression(expr_name="z2", expr=casadi.tanh(50*casadi.fmax(0, r0_y-(self.Y/3))))
        z2 = model.aux["z2"]
        model.set_expression(expr_name="z3", expr=casadi.tanh(50*casadi.fmax(0, r0_y-(2*(self.Y/3)))))
        z3 = model.aux["z3"]

        model.set_expression(expr_name="y_error", expr=((z1*(1-z2)*(1-z3)*y1 + z2*(1-z3)*y2 + z3*y3) - r0_y))
        y_error = model.aux["y_error"]

        model.set_rhs("y1", y1 + y_vel * self.dt)
        model.set_rhs("y2", y2 + y_vel * self.dt)
        model.set_rhs("y3", y3 + y_vel * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        model.set_rhs("r1_y", r1_y)

        model.set_expression(expr_name="lagrange_term", expr=input_weight * (y_vel**2) + position_error_weight * (y_error) ** 2)
        model.set_expression(expr_name="meyer_term", expr=position_error_weight * (y_error) ** 2)
        model.setup()
        print("Model defined!")

        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.settings.n_horizon = prediction_steps
        self.mpc.settings.t_step = self.dt
        # Required so self.mpc.data.prediction() returns the full predicted
        # trajectory instead of throwing "Optimal trajectory is not stored".
        self.mpc.settings.store_full_solution = True

        lterm = model.aux["lagrange_term"]
        mterm = model.aux["meyer_term"]
        self.mpc.set_objective(lterm=lterm, mterm=mterm)

        self.mpc.set_rterm(y_vel=input_rate_weight)

        self.mpc.bounds["lower", "_u", "y_vel"] = -v_max
        self.mpc.bounds["upper", "_u", "y_vel"] = v_max

        self.mpc.bounds["lower", "_x", "y1"] = self.Y_MIN + self.W_D - self.zone_padding
        self.mpc.bounds["upper", "_x", "y1"] = self.Y_MIN + self.W_D + self.Z_D
        self.mpc.bounds["lower", "_x", "y2"] = (self.Y/3) - self.zone_padding
        self.mpc.bounds["upper", "_x", "y2"] = 2*(self.Y/3) + self.zone_padding
        self.mpc.bounds["lower", "_x", "y3"] = self.Y_MAX - (self.W_D + self.Z_D)
        self.mpc.bounds["upper", "_x", "y3"] = self.Y_MAX - self.W_D + self.zone_padding
        self.mpc.bounds["lower", "_x", "r0_y"] = self.Y_MIN
        self.mpc.bounds["upper", "_x", "r0_y"] = self.Y_MAX

        suppress_ipopt = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())

    def recording_cmd_callback(self, msg):
        new_state = bool(msg.data)
        if new_state and not self.record_enabled:
            self.record_enabled = True
            self.segment_start_t = rospy.get_rostime().to_sec()
            self.history_entries = []
            rospy.loginfo(f"Y solver: recording STARTED at t={self.segment_start_t:.3f}")
        elif not new_state and self.record_enabled:
            self.record_enabled = False
            self.save_segment()
            rospy.loginfo("Y solver: recording STOPPED, segment saved.")

    def save_segment(self):
        if not self.history_entries:
            rospy.loginfo("Y solver: segment empty, nothing saved.")
            return
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = os.path.join(self.save_dir, f'y_solver_history_{ts}.json')
            metadata = self._collect_metadata()
            metadata['segment_start_t'] = self.segment_start_t
            metadata['duration_seconds'] = self.history_entries[-1]['t'] - self.history_entries[0]['t']
            metadata['total_entries'] = len(self.history_entries)
            payload = { 'metadata': metadata, 'solver': 'y', 'entries': self.history_entries }
            with open(fname, 'w') as f:
                json.dump(payload, f)
            rospy.loginfo(f"Y solver: saved {len(self.history_entries)} entries to {fname}")
        except Exception as e:
            rospy.logwarn(f"Y solver: save failed: {e}")

    def publish_horizon(self, t_sec):
        """
        Publish full prediction horizon.
        Wire layout: [t, y1_0,y2_0,y3_0,r0y_0,r1y_0,  y1_1,...,r1y_N,  yvel_0,...,yvel_{N-1}]
        Total length: 1 + 5*(N+1) + 1*N = 6 + 6N

        Returns a structured dict for history JSON:
          { 't', 'N', 'states': [ {'y1','y2','y3','r0_y','r1_y'}, ...(N+1)... ],
            'inputs': [ {'y_vel'}, ...(N)... ] }
        """
        try:
            y1_pred = np.array(self.mpc.data.prediction(('_x', 'y1'))).flatten()
            y2_pred = np.array(self.mpc.data.prediction(('_x', 'y2'))).flatten()
            y3_pred = np.array(self.mpc.data.prediction(('_x', 'y3'))).flatten()
            r0y_pred = np.array(self.mpc.data.prediction(('_x', 'r0_y'))).flatten()
            r1y_pred = np.array(self.mpc.data.prediction(('_x', 'r1_y'))).flatten()
            yvel_pred = np.array(self.mpc.data.prediction(('_u', 'y_vel'))).flatten()

            N = self.N_horizon
            y1_pred = y1_pred[:N+1]; y2_pred = y2_pred[:N+1]; y3_pred = y3_pred[:N+1]
            r0y_pred = r0y_pred[:N+1]; r1y_pred = r1y_pred[:N+1]; yvel_pred = yvel_pred[:N]

            if len(y1_pred) != N+1 or len(yvel_pred) != N:
                rospy.logwarn_throttle(5.0, f"Y horizon shape mismatch: y1={len(y1_pred)} yvel={len(yvel_pred)} expected N={N}")
                return None

            wire = [float(t_sec)]
            for i in range(N+1):
                wire.extend([float(y1_pred[i]), float(y2_pred[i]), float(y3_pred[i]),
                             float(r0y_pred[i]), float(r1y_pred[i])])
            for j in range(N):
                wire.append(float(yvel_pred[j]))

            msg = Float64MultiArray()
            msg.data = wire
            self.horizon_pub.publish(msg)

            # Structured form for history JSON
            states = [
                { 'y1': float(y1_pred[i]), 'y2': float(y2_pred[i]), 'y3': float(y3_pred[i]),
                  'r0_y': float(r0y_pred[i]), 'r1_y': float(r1y_pred[i]) }
                for i in range(N+1)
            ]
            inputs = [ { 'y_vel': float(yvel_pred[j]) } for j in range(N) ]
            return { 't': float(t_sec), 'N': int(N), 'states': states, 'inputs': inputs }

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Y horizon publish failed: {e}")
            return None

    def record_history(self, t_sec, state_dict, u_opt_dict, horizon_payload):
        entry = { 't': t_sec, 'state': state_dict, 'u_opt': u_opt_dict, 'horizon': horizon_payload }
        self.history_entries.append(entry)
        if len(self.history_entries) > self.max_history:
            self.history_entries.pop(0)

    def save_history_on_shutdown(self):
        """
        On shutdown (Ctrl-C), if recording is still active, save the
        segment as-is. Equivalent to pressing stop before shutting down.
        """
        if not self.record_enabled:
            rospy.loginfo("Y solver: shutdown with no active recording.")
            return
        rospy.loginfo("Y solver: shutdown during recording, saving segment.")
        self.save_segment()

    def _collect_metadata(self):
        def safe_get(path, default=None):
            try: return rospy.get_param(path)
            except: return default
        return {
            'solver': 'y',
            'saved_at': datetime.datetime.now().isoformat(),
            'N_horizon': self.N_horizon,
            'config': {
                'field_width': safe_get('/table_measurements/field_width'),
                'field_height': safe_get('/table_measurements/field_height'),
                'player_length': safe_get('/table_measurements/player_length'),
                'player_height': safe_get('/table_measurements/player_height'),
                'distance_between_players': safe_get('/table_measurements/distance_between_players'),
                'player_distance_from_wall': safe_get('/table_measurements/player_distance_from_wall'),
                'distance_to_clear_zone': safe_get('/table_measurements/distance_to_clear_zone'),
                'ball_diameter': safe_get('/table_measurements/ball_diameter'),
                'rod_one_x': safe_get('/table_measurements/blue_rod_positions/rod_one'),
                'rod_three_x': safe_get('/table_measurements/blue_rod_positions/rod_three'),
            },
            'solver_config': {
                'y_timestep': safe_get('/y_solver_parameters/timestep'),
                'y_prediction_steps': safe_get('/y_solver_parameters/prediction_steps'),
                'y_max_input': safe_get('/y_solver_parameters/max_input'),
                'y_input_rate_weight': safe_get('/y_solver_parameters/input_rate_weight'),
                'y_input_weight': safe_get('/y_solver_parameters/cost_function/input_weight'),
                'y_position_error_weight': safe_get('/y_solver_parameters/cost_function/position_error_weight'),
                'y_velocity_error_weight': safe_get('/y_solver_parameters/cost_function/velocity_error_weight'),
            },
        }

    def start(self):
        print("Starting control loop...")
        while not rospy.is_shutdown():
            if self.ball_flag == True:
                y1 = self.motor1_pos[0]
                y2 = self.motor1_pos[1]
                y3 = self.motor1_pos[2]
                r0_y = float(self.ball_pos.linear.y)
                r1_y = float(self.ball_pos.angular.y)

                u_opt = self.mpc.make_step(np.array([y1, y2, y3, r0_y, r1_y]))

                self.rod_vel = u_opt[0][0]
                self.omega_cmd.linear.x = self.rod_vel * self.radians_per_meter
                self.cmd_pub.publish(self.omega_cmd)

                # NEW: publish horizon and record history
                t_sec = rospy.get_rostime().to_sec()
                horizon_payload = self.publish_horizon(t_sec)
                if self.record_enabled:
                    self.record_history(
                        t_sec,
                        { 'y1': float(y1), 'y2': float(y2), 'y3': float(y3),
                          'r0_y': float(r0_y), 'r1_y': float(r1_y) },
                        { 'y_vel': float(self.rod_vel),
                          'omega_linear_rad_s': float(self.omega_cmd.linear.x) },
                        horizon_payload,
                    )

            self.rate.sleep()


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()
