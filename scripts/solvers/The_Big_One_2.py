#!/usr/bin/env python3
"""
Combined X/Y MPC solver.

This file stitches together the previously-separate MPC_X_Solver_Metric_Reduced_Unbounded.py
and MPC_Y_Solver_Metric_Reduced_v4.py into a single MPC controller with a
combined state vector and combined input vector:

  state  = [y1, y2, y3, r0_y, r1_y,  theta, r0_x, r1_x]   (8 entries)
  input  = [y_vel, omega]                                  (2 entries)

The combined cost function is the sum of the two original cost functions.
No individual Y-direction or X-direction term has been modified. Because both
directions now share one optimization problem, cross-direction cost terms
(e.g. penalties that mix r0_x and r0_y, or omega and y_vel) can be added
inside init_mpc() without any further restructuring.

All ROS1 integration is preserved verbatim:
  Subscriptions:
    /ball_pos                (geometry_msgs/Twist)
    /rod1_player_positions   (std_msgs/Float64MultiArray)   -- Y solver input
    /rod2_player_positions   (std_msgs/Float64MultiArray)   -- X solver input
    /recording/cmd           (std_msgs/Bool)
  Publications:
    /omega_d_y               (geometry_msgs/Twist)
    /omega_d_x               (geometry_msgs/Twist)
    /collision_gain_val      (std_msgs/Float32)
    /distance_val            (std_msgs/Float32)
    /x_solver/horizon        (std_msgs/Float64MultiArray)
    /y_solver/horizon        (std_msgs/Float64MultiArray)

Timestep and prediction horizon:
  A single MPC needs a single dt and a single n_horizon. The combined solver
  uses /x_solver_parameters/timestep and /x_solver_parameters/prediction_steps
  as the authoritative values, and warns if the Y-side params disagree. Both
  original parameter trees are still read so nothing downstream breaks.
"""

import do_mpc
from casadi import *
from casadi.tools import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import rospy
import numpy as np
import math
import json
import os
import datetime


class MPC_Solver:

    def __init__(self):

        rospy.init_node('mpc_xy_solver', anonymous=True)

        # --- Subscriptions (both solvers' inputs) ---
        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        rospy.Subscriber("/rod1_player_positions", Float64MultiArray, self.player_callback, queue_size=10)
        rospy.Subscriber("/rod2_player_positions", Float64MultiArray, self.angular_callback, queue_size=10)
        rospy.Subscriber('/recording/cmd', Bool, self.recording_cmd_callback, queue_size=10)

        # --- Publications (both solvers' outputs) ---
        self.cmd_pub_y = rospy.Publisher('/omega_d_y', Twist, queue_size=10)
        self.cmd_pub_x = rospy.Publisher('/omega_d_x', Twist, queue_size=10)
        self.gain_pub = rospy.Publisher('/collision_gain_val', Float32, queue_size=10)
        self.dist_pub = rospy.Publisher('/distance_val', Float32, queue_size=10)
        self.horizon_pub_y = rospy.Publisher('/y_solver/horizon', Float64MultiArray, queue_size=10)
        self.horizon_pub_x = rospy.Publisher('/x_solver/horizon', Float64MultiArray, queue_size=10)

        # -------- Timestep reconciliation --------
        # A combined MPC has a single dt. Use the X-side as authoritative and
        # warn if the Y-side timestep disagrees.
        self.dt = rospy.get_param('/x_solver_parameters/timestep')
        y_dt = rospy.get_param('/y_solver_parameters/timestep')
        if abs(y_dt - self.dt) > 1e-9:
            rospy.logwarn(
                f"XY solver: /x_solver_parameters/timestep ({self.dt}) "
                f"disagrees with /y_solver_parameters/timestep ({y_dt}). "
                f"Using x-side value; update params to match.")
        self.rate = rospy.Rate(int(1 / self.dt))

        # -------- X-solver-specific member state --------
        self.k = rospy.get_param('/x_solver_parameters/k_value')
        self.e = 0.85

        self.ball_flag = False
        self.rad_flag = False
        self.ball_pos = Twist()
        self.motor1_pos = Float64MultiArray()   # rod1 / y-direction players
        self.motor2_rad = Float64MultiArray()   # rod2 / x-direction angle

        self.omega_cmd_y = Twist()
        self.omega_cmd_x = Twist()
        self.rod_vel = 0
        self.rod_angular_vel = 0

        # -------- Table geometry (both directions) --------
        self.X = rospy.get_param('/table_measurements/field_width')
        self.X_MAX = self.X
        self.X_MIN = 0
        self.X_ROD = rospy.get_param('/table_measurements/blue_rod_positions/rod_one') - 0.02

        self.L_P = rospy.get_param('/table_measurements/player_length')
        self.H_P = rospy.get_param('/table_measurements/player_height')
        self.R_F = rospy.get_param('/table_measurements/approximate_player_foot_diameter') / 2
        self.R_B = rospy.get_param('/table_measurements/ball_diameter') / 2

        self.P_D = rospy.get_param('/table_measurements/distance_between_players')
        self.Z_D = rospy.get_param('/table_measurements/distance_to_clear_zone')
        self.W_D = rospy.get_param('/table_measurements/player_distance_from_wall')
        self.zone_padding = 0.05

        self.Y = rospy.get_param('/table_measurements/field_height')
        self.Y_MAX = self.Y
        self.Y_MIN = 0

        self.steps_per_revolution = rospy.get_param('/table_measurements/steps_per_revolution')
        self.meters_per_step = self.Y / rospy.get_param('/table_measurements/steps_across_field')
        self.radians_per_meter = math.pow(self.meters_per_step, -1) * (2 * math.pi / self.steps_per_revolution)
        self.meters_per_radian = math.pow(self.radians_per_meter, -1)

        # -------- History / recording (shared across both directions) --------
        self.history_entries = []
        self.max_history = 100000
        # Allow either solver's save_dir param; X takes precedence.
        self.save_dir = rospy.get_param('/x_solver_parameters/history_save_dir',
                        rospy.get_param('/y_solver_parameters/history_save_dir', '/tmp'))

        self.record_enabled = False
        self.segment_start_t = None
        rospy.loginfo("XY solver: recording controlled by /recording/cmd topic (initially OFF)")

        rospy.on_shutdown(self.save_history_on_shutdown)

    # ------------------------------------------------------------------
    # Callbacks (verbatim from the originals)
    # ------------------------------------------------------------------
    def ball_callback(self, msg):
        if self.ball_flag == False:
            self.ball_flag = True
        self.ball_pos = msg

    def player_callback(self, msg):
        # Y solver's rod1 players
        self.motor1_pos = msg.data

    def angular_callback(self, msg):
        # X solver's rod2 angle
        offset = math.pi * 0.05
        self.motor2_rad = msg.data[0] + offset
        if self.rad_flag == False:
            self.rad_flag = True

    # ------------------------------------------------------------------
    # MPC setup
    # ------------------------------------------------------------------
    def init_mpc(self):

        # -------- X-solver params --------
        x_input_weight = rospy.get_param('/x_solver_parameters/cost_function/input_weight')
        x_input_rate_weight = rospy.get_param('/x_solver_parameters/input_rate_weight')
        x_position_error_weight = rospy.get_param('/x_solver_parameters/cost_function/position_error_weight')
        x_velocity_error_weight = rospy.get_param('/x_solver_parameters/cost_function/velocity_error_weight')
        ball_vel_weight = rospy.get_param('/x_solver_parameters/cost_function/ball_vel_weight')
        goal_error_weight = rospy.get_param('/x_solver_parameters/cost_function/goal_error_weight')
        resting_weight = rospy.get_param('/x_solver_parameters/cost_function/resting_weight')
        omega_max = rospy.get_param('/x_solver_parameters/max_input')
        collision_gain_type = rospy.get_param('/x_solver_parameters/collision_gain_type')
        x_prediction_steps = rospy.get_param('/x_solver_parameters/prediction_steps')

        # -------- Y-solver params --------
        y_input_weight = rospy.get_param('/y_solver_parameters/cost_function/input_weight')
        y_input_rate_weight = rospy.get_param('/y_solver_parameters/input_rate_weight')
        y_position_error_weight = rospy.get_param('/y_solver_parameters/cost_function/position_error_weight')
        y_velocity_error_weight = rospy.get_param('/y_solver_parameters/cost_function/velocity_error_weight')
        y_prediction_steps = rospy.get_param('/y_solver_parameters/prediction_steps')
        v_max = rospy.get_param('/y_solver_parameters/max_input') * self.meters_per_radian

        # -------- Horizon reconciliation --------
        prediction_steps = x_prediction_steps
        if y_prediction_steps != x_prediction_steps:
            rospy.logwarn(
                f"XY solver: X prediction_steps ({x_prediction_steps}) "
                f"disagrees with Y prediction_steps ({y_prediction_steps}). "
                f"Using X-side value for combined horizon.")
        self.N_horizon = prediction_steps

        print("Combined XY Solver Parameters")
        print("-----------------------------")
        print("Prediction Horizon:", prediction_steps)
        print("Timestep:", self.dt)
        print("-- X-direction --")
        print("  Input Weight:", x_input_weight)
        print("  Position Error Weight:", x_position_error_weight)
        print("  Velocity Error Weight:", x_velocity_error_weight)
        print("  Input Rate Weight:", x_input_rate_weight)
        print("  Goal Error Weight:", goal_error_weight)
        print("  Resting Weight:", resting_weight)
        print("  Ball Vel Weight:", ball_vel_weight)
        print("  Collision Gain Type:", collision_gain_type)
        print("  omega_max:", omega_max)
        print("-- Y-direction --")
        print("  Input Weight:", y_input_weight)
        print("  Position Error Weight:", y_position_error_weight)
        print("  Velocity Error Weight:", y_velocity_error_weight)
        print("  Input Rate Weight:", y_input_rate_weight)
        print("  v_max:", v_max, "\n")

        # =================================================================
        # Model — combined state & input
        # =================================================================
        model = do_mpc.model.Model('discrete')

        # --- Y-direction states (verbatim from Y solver) ---
        y1 = model.set_variable(var_type='_x', var_name='y1', shape=(1, 1))
        y2 = model.set_variable(var_type='_x', var_name='y2', shape=(1, 1))
        y3 = model.set_variable(var_type='_x', var_name='y3', shape=(1, 1))
        r0_y = model.set_variable(var_type='_x', var_name='r0_y', shape=(1, 1))
        r1_y = model.set_variable(var_type='_x', var_name='r1_y', shape=(1, 1))

        # --- X-direction states (verbatim from X solver) ---
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))

        # --- Inputs ---
        y_vel = model.set_variable(var_type='_u', var_name='y_vel')
        omega = model.set_variable(var_type='_u', var_name='omega')

        # =================================================================
        # Y-direction auxiliary expressions and error (verbatim)
        # =================================================================
        model.set_expression(expr_name="z1", expr=casadi.tanh(50 * casadi.fmax(0, r0_y)))
        z1 = model.aux["z1"]
        model.set_expression(expr_name="z2", expr=casadi.tanh(50 * casadi.fmax(0, r0_y - (self.Y / 3))))
        z2 = model.aux["z2"]
        model.set_expression(expr_name="z3", expr=casadi.tanh(50 * casadi.fmax(0, r0_y - (2 * (self.Y / 3)))))
        z3 = model.aux["z3"]

        model.set_expression(
            expr_name="y_nearest",
            expr=(z1 * (1 - z2) * (1 - z3) * y1 + z2 * (1 - z3) * y2 + z3 * y3) )
        y_nearest = model.aux["y_nearest"]
        model.set_expression(
            expr_name="y_error",
            expr=(y_nearest - r0_y) )
        y_error = model.aux["y_error"]
 
        # =================================================================
        # X-direction auxiliary expressions and collision gain (verbatim)
        # =================================================================
        model.set_expression(expr_name="x_foot", expr=self.X_ROD + self.L_P * casadi.sin(theta))
        x_foot = model.aux["x_foot"]
        model.set_expression(expr_name="z_foot", expr=self.L_P * casadi.cos(theta))
        z_foot = model.aux["z_foot"]
        model.set_expression(expr_name="r0_z", expr=casadi.SX(self.H_P - self.R_B))
        r0_z = model.aux["r0_z"]
        #model.set_expression(expr_name="dist",
        #                     expr=casadi.sqrt((x_foot - r0_x) ** 2 + (z_foot - r0_z) ** 2 + 1e-6))
        #dist = model.aux["dist"]
        model.set_expression(expr_name="dist",
                             expr=casadi.sqrt((x_foot - r0_x) ** 2 + (z_foot - r0_z) ** 2 + (y_nearest - r0_y) ** 2 + 1e-6))
        dist = model.aux["dist"]
        model.set_expression(expr_name="v_foot_x", expr=omega * self.L_P * casadi.cos(theta))
        v_foot_x = model.aux["v_foot_x"]

        model.set_expression(expr_name="zm1",
                             expr=casadi.tanh(50 * casadi.fmax(0, r0_x - 0))) 
        zm1 = model.aux["zm1"]
        model.set_expression(expr_name="z0",
                             expr=casadi.tanh(50 * casadi.fmax(0, r0_x - (self.X_ROD - 0.00)))) # 0.03
        z0 = model.aux["z0"]
        # NOTE: X solver originally named this "z1" too; we rename to "zx1" here
        # because the Y solver already occupies the name "z1" in the shared model.
        # The scalar expression is unchanged.
        model.set_expression(expr_name="zx1",
                             expr=casadi.tanh(50 * casadi.fmax(0, r0_x - (self.X_ROD + 0.06)))) # 0.03
        zx1 = model.aux["zx1"]

        model.set_expression(expr_name="c0", expr=z0 * (1 - zx1))
        c0 = model.aux["c0"]
        model.set_expression(expr_name="c1", expr=(1 - c0))
        c1 = model.aux["c1"]
        model.set_expression(expr_name="cm1", expr=zm1 * (1 - c1) * (1 - c0))
        cm1 = model.aux["cm1"]
        
        if collision_gain_type == 1:
            model.set_expression(expr_name="collision_gain_1",
                expr=casadi.tanh(200 * casadi.fmax(0, (self.R_F*0.7 + self.R_B) - casadi.fabs(dist))))
            collision_gain = model.aux["collision_gain_1"]
        elif collision_gain_type == 2:
            model.set_expression(expr_name="collision_gain_2",
                expr=(x_foot - r0_x) / (self.X_MAX - x_foot))
            collision_gain = model.aux["collision_gain_2"]
        elif collision_gain_type == 3:
            model.set_expression(expr_name="collision_gain_3",
                expr=(1 / ((1 / 10) * casadi.fabs(x_foot - r0_x)) ** 100 + 1))
            collision_gain = model.aux["collision_gain_3"]
        elif collision_gain_type == 4:
            model.set_expression(expr_name="collision_gain_4",
                expr=(1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist)))))
            collision_gain = model.aux["collision_gain_4"]
        elif collision_gain_type == 5:
            model.set_expression(expr_name="collision_gain_5",
                expr=(1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist))))
                     * (1 / 1 + casadi.exp(-150 * (x_foot - r0_x))))
            collision_gain = model.aux["collision_gain_5"]

        print("Collision Gain Equation (Type " + str(collision_gain_type) + "): ", collision_gain, "\n")

        # =================================================================
        # Dynamics (verbatim Y then verbatim X)
        # =================================================================
        # Y dynamics
        model.set_rhs("y1", y1 + y_vel * self.dt)
        model.set_rhs("y2", y2 + y_vel * self.dt)
        model.set_rhs("y3", y3 + y_vel * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        #model.set_rhs("r1_y", r1_y)
        model.set_rhs("r1_y", (1 - collision_gain) * r1_y + (collision_gain) * (-self.e * r1_y + (1 + self.e) * y_vel))
        
        # X dynamics
        model.set_rhs("theta", theta + omega * self.dt)
        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        #model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain) * self.L_P * omega * casadi.cos(theta))
        model.set_rhs("r1_x", (1 - collision_gain) * r1_x + (collision_gain) * (-self.e * r1_x + (1 + self.e) * v_foot_x))
        #model.set_rhs("r1_x", r1_x)

        # =================================================================
        # Combined cost function
        #
        # Lagrange and Mayer terms are the SUM of the two originals. Neither
        # individual term has been edited. New coupling terms that mix X and
        # Y states/inputs can be added to `combined_lagrange` or
        # `combined_meyer` below.
        # =================================================================

        # --- Y lagrange/meyer (verbatim) ---
        y_lagrange = (y_input_weight * (y_vel ** 2) )
                      #+ y_position_error_weight * (y_error) ** 2
                      #+ goal_error_weight * ((r0_y - (self.Y_MAX / 2))) ** 2 )
        y_meyer = casadi.SX(0) #+ y_position_error_weight * (y_error) ** 2

        # --- X lagrange/meyer (verbatim, commented lines preserved) ---
        x_lagrange = (
            x_input_weight * omega ** 2
            #+ goal_error_weight * ((r0_x - 5 * self.X_MAX)) ** 2
        )
        x_meyer = (casadi.SX(0)
            #+ ball_vel_weight * ((1 / (r1_x + 0.1))) ** 2
        )

        # ---------------------------------------------------------------
        # Add cross-direction / coupling cost terms below. Anything added
        # here will see both the X and Y state variables simultaneously.
        # Example:
        #   coupling = some_weight * ((r0_x - x_foot)*(r0_y - y2))**2
        # ---------------------------------------------------------------
        coupling_lagrange = casadi.SX(0) + goal_error_weight / ( (r0_x - self.X_ROD) + 0.001) ** 2 # + goal_error_weight / (r0_x**2 + r0_y**2 + 0.001) #+ goal_error_weight * ((dist * omega) * zm1) ** 2 #+ goal_error_weight * ( 1 / (casadi.sqrt((r0_x - 0) ** 2 + (r0_y - 0) **2 + 1e-6) + 1e-1)) ** 2
        coupling_meyer = casadi.SX(0) #+ goal_error_weight * ( ((r0_x - 5 * self.X_MAX)) ** 2 + (r0_y - (self.Y_MAX/2) ) ** 2 )

        combined_lagrange = y_lagrange + x_lagrange + coupling_lagrange
        combined_meyer = y_meyer + x_meyer + coupling_meyer

        model.set_expression(expr_name="lagrange_term", expr=combined_lagrange)
        model.set_expression(expr_name="meyer_term", expr=combined_meyer)

        # Also expose the per-direction pieces as aux for logging / analysis.
        model.set_expression(expr_name="y_lagrange", expr=y_lagrange)
        model.set_expression(expr_name="y_meyer", expr=y_meyer)
        model.set_expression(expr_name="x_lagrange", expr=x_lagrange)
        model.set_expression(expr_name="x_meyer", expr=x_meyer)

        print("Solver Status")
        print("-----------------------------")
        model.setup()
        print("Model defined!")

        # =================================================================
        # Controller
        # =================================================================
        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.settings.n_horizon = prediction_steps
        self.mpc.settings.t_step = self.dt
        self.mpc.settings.store_full_solution = True

        lterm = model.aux["lagrange_term"]
        mterm = model.aux["meyer_term"]
        self.mpc.set_objective(lterm=lterm, mterm=mterm)

        # Per-input rate weights (each keeps its original value)
        self.mpc.set_rterm(y_vel=y_input_rate_weight, omega=x_input_rate_weight)

        # --- Input bounds (verbatim from each original) ---
        self.mpc.bounds["lower", "_u", "y_vel"] = -v_max
        self.mpc.bounds["upper", "_u", "y_vel"] = v_max
        self.mpc.bounds["lower", "_u", "omega"] = -omega_max
        self.mpc.bounds["upper", "_u", "omega"] = omega_max

        # --- Y state bounds (verbatim) ---
        self.mpc.bounds["lower", "_x", "y1"] = self.Y_MIN + self.W_D - self.zone_padding
        self.mpc.bounds["upper", "_x", "y1"] = self.Y_MIN + self.W_D + self.Z_D
        self.mpc.bounds["lower", "_x", "y2"] = (self.Y / 3) - self.zone_padding
        self.mpc.bounds["upper", "_x", "y2"] = 2 * (self.Y / 3) + self.zone_padding
        self.mpc.bounds["lower", "_x", "y3"] = self.Y_MAX - (self.W_D + self.Z_D)
        self.mpc.bounds["upper", "_x", "y3"] = self.Y_MAX - self.W_D + self.zone_padding
        self.mpc.bounds["lower", "_x", "r0_y"] = self.Y_MIN
        self.mpc.bounds["upper", "_x", "r0_y"] = self.Y_MAX

        # --- X state bounds (verbatim; r0_x bounds were commented out in the original) ---
        self.mpc.bounds["lower", "_x", "theta"] = -casadi.pi / 2
        self.mpc.bounds["upper", "_x", "theta"] = casadi.pi / 2
        #self.mpc.bounds["lower", "_x", "r0_x"] = self.X_MIN
        #self.mpc.bounds["upper", "_x", "r0_x"] = self.X_MAX

        suppress_ipopt = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        # State order: [y1, y2, y3, r0_y, r1_y, theta, r0_x, r1_x]
        self.mpc.x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())

    # ------------------------------------------------------------------
    # Recording / history (merged from both originals)
    # ------------------------------------------------------------------
    def recording_cmd_callback(self, msg):
        new_state = bool(msg.data)
        if new_state and not self.record_enabled:
            self.record_enabled = True
            self.segment_start_t = rospy.get_rostime().to_sec()
            self.history_entries = []
            rospy.loginfo(f"XY solver: recording STARTED at t={self.segment_start_t:.3f}")
        elif not new_state and self.record_enabled:
            self.record_enabled = False
            self.save_segment()
            rospy.loginfo("XY solver: recording STOPPED, segment saved.")

    def save_segment(self):
        if not self.history_entries:
            rospy.loginfo("XY solver: segment empty, nothing saved.")
            return
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = os.path.join(self.save_dir, f'xy_solver_history_{ts}.json')
            metadata = self._collect_metadata()
            metadata['segment_start_t'] = self.segment_start_t
            metadata['duration_seconds'] = self.history_entries[-1]['t'] - self.history_entries[0]['t']
            metadata['total_entries'] = len(self.history_entries)
            payload = {'metadata': metadata, 'solver': 'xy', 'entries': self.history_entries}
            with open(fname, 'w') as f:
                json.dump(payload, f)
            rospy.loginfo(f"XY solver: saved {len(self.history_entries)} entries to {fname}")
        except Exception as e:
            rospy.logwarn(f"XY solver: save failed: {e}")

    def save_history_on_shutdown(self):
        if not self.record_enabled:
            rospy.loginfo("XY solver: shutdown with no active recording.")
            return
        rospy.loginfo("XY solver: shutdown during recording, saving segment.")
        self.save_segment()

    # ------------------------------------------------------------------
    # Horizon publishing (two topics, same wire format as the originals)
    # ------------------------------------------------------------------
    def publish_horizons(self, t_sec):
        """
        Publish the X and Y slices of the combined prediction on their
        original topics, preserving the wire format used by the two
        standalone solvers.

        X wire layout: [t, theta_0, r0x_0, r1x_0,  theta_1,..., r1x_N,  omega_0,...,omega_{N-1}]
          Total length: 1 + 3*(N+1) + 1*N = 4 + 4N
        Y wire layout: [t, y1_0,y2_0,y3_0,r0y_0,r1y_0,  y1_1,...,r1y_N,  yvel_0,...,yvel_{N-1}]
          Total length: 1 + 5*(N+1) + 1*N = 6 + 6N

        Returns a structured dict containing both x and y horizons for the
        history JSON.
        """
        horizon_x = None
        horizon_y = None

        # ---- X slice ----
        try:
            theta_pred = np.array(self.mpc.data.prediction(('_x', 'theta'))).flatten()
            r0x_pred = np.array(self.mpc.data.prediction(('_x', 'r0_x'))).flatten()
            r1x_pred = np.array(self.mpc.data.prediction(('_x', 'r1_x'))).flatten()
            omega_pred = np.array(self.mpc.data.prediction(('_u', 'omega'))).flatten()

            N = self.N_horizon
            theta_pred = theta_pred[:N + 1]
            r0x_pred = r0x_pred[:N + 1]
            r1x_pred = r1x_pred[:N + 1]
            omega_pred = omega_pred[:N]

            if len(theta_pred) != N + 1 or len(omega_pred) != N:
                rospy.logwarn_throttle(5.0,
                    f"X horizon shape mismatch: theta={len(theta_pred)} omega={len(omega_pred)} expected N={N}")
            else:
                wire = [float(t_sec)]
                for i in range(N + 1):
                    wire.extend([float(theta_pred[i]), float(r0x_pred[i]), float(r1x_pred[i])])
                for j in range(N):
                    wire.append(float(omega_pred[j]))

                msg = Float64MultiArray()
                msg.data = wire
                self.horizon_pub_x.publish(msg)

                states = [
                    {'theta': float(theta_pred[i]), 'r0_x': float(r0x_pred[i]), 'r1_x': float(r1x_pred[i])}
                    for i in range(N + 1)
                ]
                inputs = [{'omega': float(omega_pred[j])} for j in range(N)]
                horizon_x = {'t': float(t_sec), 'N': int(N), 'states': states, 'inputs': inputs}

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"X horizon publish failed: {e}")

        # ---- Y slice ----
        try:
            y1_pred = np.array(self.mpc.data.prediction(('_x', 'y1'))).flatten()
            y2_pred = np.array(self.mpc.data.prediction(('_x', 'y2'))).flatten()
            y3_pred = np.array(self.mpc.data.prediction(('_x', 'y3'))).flatten()
            r0y_pred = np.array(self.mpc.data.prediction(('_x', 'r0_y'))).flatten()
            r1y_pred = np.array(self.mpc.data.prediction(('_x', 'r1_y'))).flatten()
            yvel_pred = np.array(self.mpc.data.prediction(('_u', 'y_vel'))).flatten()

            N = self.N_horizon
            y1_pred = y1_pred[:N + 1]
            y2_pred = y2_pred[:N + 1]
            y3_pred = y3_pred[:N + 1]
            r0y_pred = r0y_pred[:N + 1]
            r1y_pred = r1y_pred[:N + 1]
            yvel_pred = yvel_pred[:N]

            if len(y1_pred) != N + 1 or len(yvel_pred) != N:
                rospy.logwarn_throttle(5.0,
                    f"Y horizon shape mismatch: y1={len(y1_pred)} yvel={len(yvel_pred)} expected N={N}")
            else:
                wire = [float(t_sec)]
                for i in range(N + 1):
                    wire.extend([float(y1_pred[i]), float(y2_pred[i]), float(y3_pred[i]),
                                 float(r0y_pred[i]), float(r1y_pred[i])])
                for j in range(N):
                    wire.append(float(yvel_pred[j]))

                msg = Float64MultiArray()
                msg.data = wire
                self.horizon_pub_y.publish(msg)

                states = [
                    {'y1': float(y1_pred[i]), 'y2': float(y2_pred[i]), 'y3': float(y3_pred[i]),
                     'r0_y': float(r0y_pred[i]), 'r1_y': float(r1y_pred[i])}
                    for i in range(N + 1)
                ]
                inputs = [{'y_vel': float(yvel_pred[j])} for j in range(N)]
                horizon_y = {'t': float(t_sec), 'N': int(N), 'states': states, 'inputs': inputs}

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Y horizon publish failed: {e}")

        return {'x': horizon_x, 'y': horizon_y}

    def record_history(self, t_sec, state_dict, u_opt_dict, aux_dict, horizon_payload):
        entry = {
            't': t_sec,
            'state': state_dict,
            'u_opt': u_opt_dict,
            'aux': aux_dict,
            'horizon': horizon_payload,
        }
        self.history_entries.append(entry)
        if len(self.history_entries) > self.max_history:
            self.history_entries.pop(0)

    def _collect_metadata(self):
        def safe_get(path, default=None):
            try:
                return rospy.get_param(path)
            except:
                return default
        return {
            'solver': 'xy',
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
                'player_foot_diameter': safe_get('/table_measurements/approximate_player_foot_diameter'),
                'rod_one_x': safe_get('/table_measurements/blue_rod_positions/rod_one'),
                'rod_three_x': safe_get('/table_measurements/blue_rod_positions/rod_three'),
                'steps_per_revolution': safe_get('/table_measurements/steps_per_revolution'),
                'steps_across_field': safe_get('/table_measurements/steps_across_field'),
                'gainType': safe_get('/x_solver_parameters/collision_gain_type'),
            },
            'solver_config': {
                'combined_timestep': self.dt,
                'combined_prediction_steps': self.N_horizon,
                'x_timestep': safe_get('/x_solver_parameters/timestep'),
                'x_prediction_steps': safe_get('/x_solver_parameters/prediction_steps'),
                'x_max_input': safe_get('/x_solver_parameters/max_input'),
                'x_input_rate_weight': safe_get('/x_solver_parameters/input_rate_weight'),
                'x_input_weight': safe_get('/x_solver_parameters/cost_function/input_weight'),
                'x_position_error_weight': safe_get('/x_solver_parameters/cost_function/position_error_weight'),
                'x_goal_error_weight': safe_get('/x_solver_parameters/cost_function/goal_error_weight'),
                'x_resting_weight': safe_get('/x_solver_parameters/cost_function/resting_weight'),
                'y_timestep': safe_get('/y_solver_parameters/timestep'),
                'y_prediction_steps': safe_get('/y_solver_parameters/prediction_steps'),
                'y_max_input': safe_get('/y_solver_parameters/max_input'),
                'y_input_rate_weight': safe_get('/y_solver_parameters/input_rate_weight'),
                'y_input_weight': safe_get('/y_solver_parameters/cost_function/input_weight'),
                'y_position_error_weight': safe_get('/y_solver_parameters/cost_function/position_error_weight'),
                'y_velocity_error_weight': safe_get('/y_solver_parameters/cost_function/velocity_error_weight'),
            },
        }

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def start(self):
        print("Starting control loop...")
        while not rospy.is_shutdown():
            # Require BOTH ball detection AND rod2 angle sample, matching the
            # gate conditions each standalone solver used.
            if self.ball_flag and self.rad_flag:
                # Y-side measurements
                y1 = self.motor1_pos[0]
                y2 = self.motor1_pos[1]
                y3 = self.motor1_pos[2]
                r0_y = float(self.ball_pos.linear.y)
                r1_y = float(self.ball_pos.angular.y)

                # X-side measurements
                theta = self.motor2_rad
                r0_x = float(self.ball_pos.linear.x)
                r1_x = float(self.ball_pos.angular.x)

                # State order MUST match model variable declaration order:
                # [y1, y2, y3, r0_y, r1_y, theta, r0_x, r1_x]
                u_opt = self.mpc.make_step(np.array([y1, y2, y3, r0_y, r1_y, theta, r0_x, r1_x]))

                # do_mpc returns inputs in declaration order: [y_vel, omega]
                self.rod_vel = u_opt[0][0]
                self.rod_angular_vel = u_opt[1][0]

                # Publish Y command (converted to rad/s on linear.x, per original)
                self.omega_cmd_y.linear.x = self.rod_vel * self.radians_per_meter
                self.cmd_pub_y.publish(self.omega_cmd_y)

                # Publish X command (rad/s on angular.x, per original)
                self.omega_cmd_x.angular.x = self.rod_angular_vel
                self.cmd_pub_x.publish(self.omega_cmd_x)

                # Publish collision gain / distance (X-side auxiliaries)
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

                # Publish horizons and record
                t_sec = rospy.get_rostime().to_sec()
                horizon_payload = self.publish_horizons(t_sec)

                if self.record_enabled:
                    self.record_history(
                        t_sec,
                        {
                            'y1': float(y1), 'y2': float(y2), 'y3': float(y3),
                            'r0_y': float(r0_y), 'r1_y': float(r1_y),
                            'theta': float(theta), 'r0_x': float(r0_x), 'r1_x': float(r1_x),
                        },
                        {
                            'y_vel': float(self.rod_vel),
                            'omega': float(self.rod_angular_vel),
                            'omega_linear_rad_s': float(self.omega_cmd_y.linear.x),
                        },
                        {
                            'gain': float(current_gain) if current_gain is not None else None,
                            'dist': float(current_dist) if current_dist is not None else None,
                        },
                        horizon_payload,
                    )

            self.rate.sleep()


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()
