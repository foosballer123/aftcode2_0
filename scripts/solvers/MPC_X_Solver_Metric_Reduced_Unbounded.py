#!/usr/bin/env python3

import do_mpc
from casadi import *
from casadi.tools import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import rospy
import numpy as np
import math
import json
import os
import datetime

class MPC_Solver:

    def __init__(self):

        rospy.init_node('mpc_x_solver', anonymous=True)

        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        rospy.Subscriber("/rod2_player_positions", Float64MultiArray, self.angular_callback, queue_size=10)
        self.cmd_pub = rospy.Publisher('/omega_d_x', Twist, queue_size=10)
        self.gain_pub = rospy.Publisher('/collision_gain_val', Float32, queue_size=10)
        self.dist_pub = rospy.Publisher('/distance_val', Float32, queue_size=10)
        # NEW: horizon publisher
        self.horizon_pub = rospy.Publisher('/x_solver/horizon', Float64MultiArray, queue_size=10)
        # NEW: recording command subscriber (from Bool /recording/cmd)
        from std_msgs.msg import Bool
        rospy.Subscriber('/recording/cmd', Bool, self.recording_cmd_callback, queue_size=10)

        self.dt = rospy.get_param('/x_solver_parameters/timestep')
        self.rate = rospy.Rate(int(1/self.dt))

        self.k = rospy.get_param('/x_solver_parameters/k_value')
        self.e = 0.85
        
        self.ball_flag = False
        self.rad_flag = False
        self.ball_pos = Twist()
        self.motor2_rad = Float64MultiArray()

        self.omega_cmd = Twist()
        self.rod_angular_vel = 0

        self.X = rospy.get_param('/table_measurements/field_width')
        self.X_MAX = self.X
        self.X_MIN = 0
        self.X_ROD = rospy.get_param('/table_measurements/blue_rod_positions/rod_one') - 0.02

        self.L_P = rospy.get_param('/table_measurements/player_length')
        self.H_P = rospy.get_param('/table_measurements/player_height')
        self.R_F = rospy.get_param('/table_measurements/approximate_player_foot_diameter') / 2
        self.R_B = rospy.get_param('/table_measurements/ball_diameter') / 2

        # FIX: self.Y was used here but removed when Y-axis variables were stripped.
        # Read field_height directly from rosparam for this calculation.
        self.steps_per_revolution = rospy.get_param('/table_measurements/steps_per_revolution')
        self.meters_per_step = rospy.get_param('/table_measurements/field_height') / rospy.get_param('/table_measurements/steps_across_field')
        self.radians_per_meter = math.pow(self.meters_per_step, -1) * (2*math.pi / self.steps_per_revolution)
        self.meters_per_radian = math.pow(self.radians_per_meter, -1)

        # NEW: history buffer for end-of-run save
        self.history_entries = []
        self.max_history = 100000  # ~ 30 minutes at 60Hz, soft cap
        self.save_dir = rospy.get_param('/x_solver_parameters/history_save_dir', '/tmp')

        # NEW: recording is controlled by /recording/cmd (Bool).
        # Off by default — twin must explicitly request recording.
        # Each stop-start cycle produces its own JSON file (Option A).
        self.record_enabled = False
        self.segment_start_t = None  # wall time of current segment start
        rospy.loginfo("X solver: recording controlled by /recording/cmd topic (initially OFF)")

        # Register shutdown hook
        rospy.on_shutdown(self.save_history_on_shutdown)

    def ball_callback(self, msg):
        if self.ball_flag == False:
            self.ball_flag = True
        self.ball_pos = msg

    def angular_callback(self, msg):
        offset = math.pi*0.05
        self.motor2_rad = msg.data[0] + offset
        if self.rad_flag == False:
            self.rad_flag = True

    def init_mpc(self):

        input_weight = rospy.get_param('/x_solver_parameters/cost_function/input_weight')
        prediction_steps = rospy.get_param('/x_solver_parameters/prediction_steps')
        input_rate_weight = rospy.get_param('/x_solver_parameters/input_rate_weight')
        position_error_weight = rospy.get_param('/x_solver_parameters/cost_function/position_error_weight')
        velocity_error_weight = rospy.get_param('/x_solver_parameters/cost_function/velocity_error_weight')
        ball_vel_weight = rospy.get_param('/x_solver_parameters/cost_function/ball_vel_weight')
        goal_error_weight = rospy.get_param('/x_solver_parameters/cost_function/goal_error_weight')
        resting_weight = rospy.get_param('/x_solver_parameters/cost_function/resting_weight')
        omega_max = rospy.get_param('/x_solver_parameters/max_input')
        collision_gain_type = rospy.get_param('/x_solver_parameters/collision_gain_type')

        # NEW: remember N for horizon packing
        self.N_horizon = prediction_steps

        print("Solver Parameters")
        print("-----------------------------")
        print("Input Weight:", input_weight)
        print("Position Error Weight:", position_error_weight)
        print("Velocity Error Weight:", velocity_error_weight)
        print("Prediction Horizon:", prediction_steps)
        print("Goal Error Weight:", goal_error_weight)
        print("Resting Weight:", resting_weight)
        print("Input Rate Weight:", input_rate_weight)
        print("Collision Gain Type:", collision_gain_type)

        model = do_mpc.model.Model('discrete')

        omega = model.set_variable(var_type='_u', var_name='omega')
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))

        model.set_rhs("theta", theta + omega * self.dt)

        model.set_expression(expr_name="x_foot", expr=self.X_ROD + self.L_P*casadi.sin(theta))
        x_foot = model.aux["x_foot"]
        model.set_expression(expr_name="z_foot", expr=self.L_P*casadi.cos(theta))
        z_foot = model.aux["z_foot"]
        model.set_expression(expr_name="r0_z", expr=casadi.SX(self.H_P - self.R_B))
        r0_z = model.aux["r0_z"]
        model.set_expression(expr_name="dist", expr=casadi.sqrt((x_foot - r0_x)**2 + (z_foot - r0_z)**2 + 1e-6))
        dist = model.aux["dist"]
        model.set_expression(expr_name="v_foot_x", expr=omega * self.L_P * casadi.cos(theta))
        v_foot_x = model.aux["v_foot_x"]

        model.set_expression(expr_name="z0", expr=casadi.tanh(50*casadi.fmax(0, r0_x - (self.X_ROD-0.01)))) 
        z0 = model.aux["z0"]
        model.set_expression(expr_name="z1", expr=casadi.tanh(50*casadi.fmax(0, r0_x - (self.X_ROD+0.03))))
        z1 = model.aux["z1"]
        
        model.set_expression(expr_name="c0", expr=z0 * (1 - z1) )
        c0 = model.aux["c0"]
        model.set_expression(expr_name="c1", expr=(1-c0) )
        c1 = model.aux["c1"]
        
        if collision_gain_type == 1:
            model.set_expression(expr_name="collision_gain_1", expr=casadi.tanh(200 * casadi.fmax(0, (self.R_F + self.R_B) - casadi.fabs(dist))))
            collision_gain = model.aux["collision_gain_1"]
        elif collision_gain_type == 2:
            model.set_expression(expr_name="collision_gain_2", expr=(x_foot-r0_x)/(self.X_MAX-x_foot))
            collision_gain = model.aux["collision_gain_2"]
        elif collision_gain_type == 3:
            model.set_expression(expr_name="collision_gain_3", expr=(1 / ((1/10)*casadi.fabs(x_foot - r0_x))**100 + 1))
            collision_gain = model.aux["collision_gain_3"]
        elif collision_gain_type == 4:
            model.set_expression(expr_name="collision_gain_4", expr=(1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist)))))
            collision_gain = model.aux["collision_gain_4"]
        elif collision_gain_type == 5:
            model.set_expression(expr_name="collision_gain_5", expr=(1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist)))) * (1 / 1 + casadi.exp(-150 * (x_foot - r0_x))))
            collision_gain = model.aux["collision_gain_5"]

        print("Collision Gain Equation (Type "+str(collision_gain_type)+"): ", collision_gain, "\n")

        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        #model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain) * self.L_P * omega * casadi.cos(theta))
        model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain) * (-self.e * r1_x + (1 + self.e) * v_foot_x))
        #model.set_rhs("r1_x", r1_x)
        
        model.set_expression(
            expr_name="lagrange_term", expr=
            input_weight * omega**2 
            #+ resting_weight * ((casadi.fmax(0, casadi.fabs(theta - (-(15/8)*casadi.pi)) - 0.5*casadi.pi)) * c1)**2
            #+ resting_weight * (casadi.fmax(0, casadi.fabs(theta - ((15/8)*casadi.pi)) - 0.1))**2 
            #+ resting_weight * (dist)**2
            #+ ball_vel_weight * (1 / (r1_x + 1e-3))**2 
            #+ goal_error_weight * (r0_x - casadi.inf)**2 
            #+ ball_vel_weight * ( (1 / (r1_x+0.1)) * c0 * (1 - c1) )**2
            #+ ball_vel_weight * ( (1 / (r1_x+0.1)) * c0 * (1 - c1) )**2
            #+ goal_error_weight * (1 / ((r0_x - 0) + 1e-6) * c0 )**2
            + goal_error_weight * ((r0_x - 5*self.X_MAX ) )**2 
            + goal_error_weight / ( (r0_x - self.X_ROD) + 0.001) ** 2
            + ball_vel_weight * ( (1 / (r1_x+0.1)) )**2
            #+ goal_error_weight * ((1 / (r0_x - 5*self.X_MAX + 1e-6)) * c0)**2 
        )
        model.set_expression(
            expr_name="meyer_term", expr=casadi.SX(0) + goal_error_weight * ((r0_x - 5*self.X_MAX ) )**2
            + resting_weight * (casadi.fmax(0, casadi.fabs(theta - (-(15/8)*casadi.pi)) - 0.5*casadi.pi))**2 
            #+ goal_error_weight * (r0_x - 5*self.X_MAX)**2 
            #+ goal_error_weight * (1 / ((r0_x - 0) + 1e-6))**2 
            #+ ball_vel_weight * ( (1 / (r1_x+0.1)) * c0 * (1 - c1) )**2
            #+ ball_vel_weight * ( (1 / (r1_x+0.1)) * c1)**2
        )

        print("Solver Status")
        print("-----------------------------")
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

        self.mpc.set_rterm(omega=input_rate_weight)

        self.mpc.bounds["lower", "_u", "omega"] = -omega_max
        self.mpc.bounds["upper", "_u", "omega"] = omega_max
        self.mpc.bounds["lower", "_x", "theta"] = -casadi.pi / 2
        self.mpc.bounds["upper", "_x", "theta"] = casadi.pi / 2
        #self.mpc.bounds["lower", "_x", "r0_x"] = self.X_MIN
        #self.mpc.bounds["upper", "_x", "r0_x"] = self.X_MAX

        suppress_ipopt = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())

    def recording_cmd_callback(self, msg):
        """
        Flips the cached record_enabled flag. This callback runs off the
        main control loop so the control loop stays deterministic —
        the flag is just a Python bool read on each iteration.
        Transitions off->on start a new segment; on->off saves it.
        """
        new_state = bool(msg.data)
        if new_state and not self.record_enabled:
            self.record_enabled = True
            self.segment_start_t = rospy.get_rostime().to_sec()
            self.history_entries = []  # fresh buffer for this segment
            rospy.loginfo(f"X solver: recording STARTED at t={self.segment_start_t:.3f}")
        elif not new_state and self.record_enabled:
            self.record_enabled = False
            self.save_segment()
            rospy.loginfo("X solver: recording STOPPED, segment saved.")

    def save_segment(self):
        """Save the current in-memory segment to JSON."""
        if not self.history_entries:
            rospy.loginfo("X solver: segment empty, nothing saved.")
            return
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = os.path.join(self.save_dir, f'x_solver_history_{ts}.json')
            metadata = self._collect_metadata()
            metadata['segment_start_t'] = self.segment_start_t
            metadata['duration_seconds'] = self.history_entries[-1]['t'] - self.history_entries[0]['t']
            metadata['total_entries'] = len(self.history_entries)
            payload = { 'metadata': metadata, 'solver': 'x', 'entries': self.history_entries }
            with open(fname, 'w') as f:
                json.dump(payload, f)
            rospy.loginfo(f"X solver: saved {len(self.history_entries)} entries to {fname}")
        except Exception as e:
            rospy.logwarn(f"X solver: save failed: {e}")

    def publish_horizon(self, t_sec):
        """
        Pack and publish the full prediction horizon.
        Wire layout: [t, theta_0, r0x_0, r1x_0,  theta_1,..., r1x_N,  omega_0,...,omega_{N-1}]
        Total length: 1 + 3*(N+1) + 1*N = 4 + 4N

        Returns a structured dict for history storage, containing the same
        data in a human-readable form:
          { 't', 'N', 'states': [ {'theta','r0_x','r1_x'}, ...(N+1 entries)... ],
            'inputs': [ {'omega'}, ...(N entries)... ] }
        """
        try:
            theta_pred = np.array(self.mpc.data.prediction(('_x', 'theta'))).flatten()
            r0x_pred   = np.array(self.mpc.data.prediction(('_x', 'r0_x'))).flatten()
            r1x_pred   = np.array(self.mpc.data.prediction(('_x', 'r1_x'))).flatten()
            omega_pred = np.array(self.mpc.data.prediction(('_u', 'omega'))).flatten()

            N = self.N_horizon
            theta_pred = theta_pred[:N+1]
            r0x_pred   = r0x_pred[:N+1]
            r1x_pred   = r1x_pred[:N+1]
            omega_pred = omega_pred[:N]

            if len(theta_pred) != N+1 or len(omega_pred) != N:
                rospy.logwarn_throttle(5.0, f"X horizon shape mismatch: theta={len(theta_pred)} omega={len(omega_pred)} expected N={N}")
                return None

            # Build the flat wire payload for the ROS topic
            wire = [float(t_sec)]
            for i in range(N+1):
                wire.extend([float(theta_pred[i]), float(r0x_pred[i]), float(r1x_pred[i])])
            for j in range(N):
                wire.append(float(omega_pred[j]))

            msg = Float64MultiArray()
            msg.data = wire
            self.horizon_pub.publish(msg)

            # Build structured form for history JSON
            states = [
                { 'theta': float(theta_pred[i]), 'r0_x': float(r0x_pred[i]), 'r1_x': float(r1x_pred[i]) }
                for i in range(N+1)
            ]
            inputs = [ { 'omega': float(omega_pred[j]) } for j in range(N) ]
            return { 't': float(t_sec), 'N': int(N), 'states': states, 'inputs': inputs }

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"X horizon publish failed: {e}")
            return None

    def record_history(self, t_sec, theta, r0x, r1x, omega_out, gain, dist, horizon_payload):
        """Append a step to the current segment's buffer."""
        entry = {
            't': t_sec,
            'state': { 'theta': float(theta), 'r0_x': float(r0x), 'r1_x': float(r1x) },
            'u_opt': { 'omega': float(omega_out) },
            'aux': { 'gain': float(gain) if gain is not None else None,
                     'dist': float(dist) if dist is not None else None },
            'horizon': horizon_payload,
        }
        self.history_entries.append(entry)
        if len(self.history_entries) > self.max_history:
            self.history_entries.pop(0)

    def save_history_on_shutdown(self):
        """
        On shutdown (Ctrl-C), if recording is still active, save the
        segment as-is. Equivalent to pressing stop before shutting down.
        """
        if not self.record_enabled:
            rospy.loginfo("X solver: shutdown with no active recording.")
            return
        rospy.loginfo("X solver: shutdown during recording, saving segment.")
        self.save_segment()

    def _collect_metadata(self):
        """Snapshot relevant params at shutdown for the saved recording."""
        def safe_get(path, default=None):
            try: return rospy.get_param(path)
            except: return default
        return {
            'solver': 'x',
            'saved_at': datetime.datetime.now().isoformat(),
            'N_horizon': self.N_horizon,
            'config': {
                'field_width': safe_get('/table_measurements/field_width'),
                'field_height': safe_get('/table_measurements/field_height'),
                'player_length': safe_get('/table_measurements/player_length'),
                'player_height': safe_get('/table_measurements/player_height'),
                'ball_diameter': safe_get('/table_measurements/ball_diameter'),
                'player_foot_diameter': safe_get('/table_measurements/approximate_player_foot_diameter'),
                'rod_one_x': safe_get('/table_measurements/blue_rod_positions/rod_one'),
                'rod_three_x': safe_get('/table_measurements/blue_rod_positions/rod_three'),
                'steps_per_revolution': safe_get('/table_measurements/steps_per_revolution'),
                'steps_across_field': safe_get('/table_measurements/steps_across_field'),
                'gainType': safe_get('/x_solver_parameters/collision_gain_type'),
            },
            'solver_config': {
                'x_timestep': safe_get('/x_solver_parameters/timestep'),
                'x_prediction_steps': safe_get('/x_solver_parameters/prediction_steps'),
                'x_max_input': safe_get('/x_solver_parameters/max_input'),
                'x_input_rate_weight': safe_get('/x_solver_parameters/input_rate_weight'),
                'x_input_weight': safe_get('/x_solver_parameters/cost_function/input_weight'),
                'x_position_error_weight': safe_get('/x_solver_parameters/cost_function/position_error_weight'),
                'x_goal_error_weight': safe_get('/x_solver_parameters/cost_function/goal_error_weight'),
                'x_resting_weight': safe_get('/x_solver_parameters/cost_function/resting_weight'),
            },
        }

    def start(self):
        print("Starting control loop...")
        while not rospy.is_shutdown():
            if self.rad_flag == True:
                theta = self.motor2_rad
                r0_x = float(self.ball_pos.linear.x)
                r1_x = float(self.ball_pos.angular.x)

                u_opt = self.mpc.make_step(np.array([theta, r0_x, r1_x]))

                gain_name = f"collision_gain_{rospy.get_param('/x_solver_parameters/collision_gain_type')}"
                current_gain = None
                current_dist = None
                try:
                    current_gain = self.mpc.data['_aux', gain_name][-1, 0]
                    current_dist = self.mpc.data['_aux', "dist"][-1, 0]
                    self.gain_pub.publish(float(current_gain))
                    self.dist_pub.publish(float(current_dist))
                except KeyError:
                    rospy.logwarn(f"Auxiliary variable {gain_name} not found in mpc.data")

                self.rod_angular_vel = u_opt[0][0]
                self.omega_cmd.angular.x = self.rod_angular_vel
                self.cmd_pub.publish(self.omega_cmd)

                # NEW: publish horizon and record history
                t_sec = rospy.get_rostime().to_sec()
                horizon_payload = self.publish_horizon(t_sec)
                if self.record_enabled:
                    self.record_history(t_sec, theta, r0_x, r1_x,
                                        self.rod_angular_vel, current_gain, current_dist,
                                        horizon_payload)

            self.rate.sleep()


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()
