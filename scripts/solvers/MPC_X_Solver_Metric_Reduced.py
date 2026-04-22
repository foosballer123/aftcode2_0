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

# TO-DO:
# 
#    - Convert pixels/second -> radians/second
#    - Define v_max (whatever 31 rad/s is in pix/s)
#    - Hope and pray this works
#    - Confirm P_D (the distance between blue players along the same rod in pixels)
#
#    - to convert radians to pixels:
#    -     find how many rotations it takes to clear the field 
#    -     radians / Y of Camera Frame = radians / pixel -> rad/pix * pix/s = rad/sec
#    -     (525 steps / ~330 pix) * (2pi radians / 400 steps) =  0.025 rad/pix
#
#    - y = 0 is away from the hardware
#    - y = 360 is closest to the hardware

# TO-DO (after 1/21/2026):
#	- It takes the linear stage 525 steps to clear ~1/3 of the y-distance of the field
#	- ~1/3 of the field in the y-direction is ~120 pixels -> 525 steps / ~120 pixels
#	- This is important for the rest of the calculations
#
# TO-DO 2/13/2026:
#	- Transition the ball variables to casadi symbolic logic
#	- Convert table measurements to metric system

# TO-DO: Add table bounds to the MPC solver
class MPC_Solver:

    def __init__(self):

        rospy.init_node('mpc_x_solver', anonymous=True)

        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        #rospy.Subscriber("/motor1_pos", Float32, self.player_callback, queue_size=10)
        rospy.Subscriber("/rod2_player_positions", Float64MultiArray, self.angular_callback, queue_size=10)
        self.cmd_pub = rospy.Publisher('/omega_d_x', Twist, queue_size=10)
        self.gain_pub = rospy.Publisher('/collision_gain_val', Float32, queue_size=10)
        self.dist_pub = rospy.Publisher('/distance_val', Float32, queue_size=10)
        
        self.dt = rospy.get_param('/x_solver_parameters/timestep')  # does not need to match the refresh rate of the camera
        self.rate = rospy.Rate(int(1/self.dt))
       
        self.k = rospy.get_param('/x_solver_parameters/k_value') # was 20, with the current setup a high k dampens the importance of a higher omega value
        
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
        
        self.steps_per_revolution = rospy.get_param('/table_measurements/steps_per_revolution')
        self.meters_per_step = self.Y / rospy.get_param('/table_measurements/steps_across_field')  
        self.radians_per_meter = math.pow(self.meters_per_step, -1) * (2*math.pi / self.steps_per_revolution)	
        self.meters_per_radian = math.pow(self.radians_per_meter, -1)
        
    def ball_callback(self, msg):
        """
        Callback function for receiving ball position data.
        Updates the variable ball_pos.
        """
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
        goal_error_weight = rospy.get_param('/x_solver_parameters/cost_function/goal_error_weight')
        resting_weight = rospy.get_param('/x_solver_parameters/cost_function/resting_weight')
        omega_max = rospy.get_param('/x_solver_parameters/max_input') # in rad/sec  
        collision_gain_type = rospy.get_param('/x_solver_parameters/collision_gain_type')            
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
        
        # input variable
        omega = model.set_variable(var_type='_u', var_name='omega')
        
        # for modeling the controlled variable
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        #x_foot = model.set_variable(var_type='_x', var_name='x_foot', shape=(1, 1)) # Does x_foot need to be a state variable if it is always based on theta?
        #z_foot = model.set_variable(var_type='_x', var_name='z_foot', shape=(1, 1))
        
        # make the ball a state variable (state variables use discrete time format x_n+1 = f(theta_n) )
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        #r0_z = model.set_variable(var_type='_x', var_name='r0_z', shape=(1, 1)) 
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))
        # no r1_Z
        
        model.set_rhs("theta", theta + omega * self.dt)
        #model.set_rhs("x_foot", self.X_ROD + self.L_P*casadi.sin(theta))
        #model.set_rhs("z_foot", self.L_P*cos(theta)) # with the torso of the player at z = 0
        
        # auxiliary variables are an algebraic projection of the current state ( x_n = f(theta_n) )
        model.set_expression(
            expr_name="x_foot", expr= self.X_ROD + self.L_P*casadi.sin(theta)
        )
        x_foot = model.aux["x_foot"]
        
        model.set_expression(
            expr_name="z_foot", expr= self.L_P*casadi.cos(theta)
        )
        z_foot = model.aux["z_foot"]
        
        model.set_expression(
            expr_name="r0_z", expr=casadi.SX(self.H_P - self.R_B)
        )
        r0_z = model.aux["r0_z"]
        
        model.set_expression(
            expr_name="dist", expr= casadi.sqrt( (x_foot - r0_x)**2 + (z_foot - r0_z)**2 + 1e-6)
        )
        dist = model.aux["dist"]
        
        if collision_gain_type == 1:
            model.set_expression(
                expr_name="collision_gain_1", expr=casadi.tanh(200 * casadi.fmax(0, (self.R_F + self.R_B) - casadi.fabs(dist)))
            )
            collision_gain = model.aux["collision_gain_1"]
            
        elif collision_gain_type == 2:
            model.set_expression(
                expr_name="collision_gain_2", expr= (x_foot-r0_x)/(self.X_MAX-x_foot)
            )
            collision_gain = model.aux["collision_gain_2"]
        
        elif collision_gain_type == 3:
            model.set_expression(
                expr_name="collision_gain_3", expr=( 1 / ((1/10)*casadi.fabs(x_foot - r0_x))**100 + 1 )
            )
            collision_gain = model.aux["collision_gain_3"]
            
        elif collision_gain_type == 4:
            model.set_expression(
                expr_name="collision_gain_4", expr= (1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist)))) 
            )
            collision_gain = model.aux["collision_gain_4"]
            
        elif collision_gain_type == 5: # collision w/ follow-through
            model.set_expression(
                expr_name="collision_gain_5", expr= (1 / (1 + casadi.exp(-300 * ((self.R_F + self.R_B) - dist)))) * (1 / 1 + casadi.exp(-50 * (x_foot - r0_x)))
            )
            collision_gain = model.aux["collision_gain_5"]
            
        print("Collision Gain Equation (Type "+str(collision_gain_type)+"): ", collision_gain, "\n")
        
        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        #model.set_rhs("r0_z", casadi.SX(self.H_P - self.R_B)) # constant
        model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain) * self.L_P * omega * casadi.cos(theta))

        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * omega**2 + resting_weight * (casadi.fmax(0, casadi.fabs(theta - ((15/8)*casadi.pi)) - 0.1))**2 + goal_error_weight * (r0_x - self.X_MAX)**2
            # the resting term includes dead-zone / dead-band
        )
        model.set_expression(
            expr_name="meyer_term", expr=resting_weight * (casadi.fmax(0, casadi.fabs(theta - ((15/8)*casadi.pi)) - 0.04))**2 + goal_error_weight * (r0_x - self.X_MAX)**2 + 25 * (1 / (r1_x + 1e-6))**2
            # extra terms: goal_error_weight * (r0_x - self.X_MAX)**2, position_error_weight * (dist) ** 2 
        )
        
        print("Solver Status")
        print("-----------------------------")
        model.setup()
        print("Model defined!")
        
        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.settings.n_horizon = prediction_steps
        self.mpc.settings.t_step = self.dt

        lterm = model.aux["lagrange_term"]
        mterm = model.aux["meyer_term"]
        self.mpc.set_objective(lterm=lterm, mterm=mterm)

        self.mpc.set_rterm(omega=input_rate_weight)
        #self.mpc.scaling['_x', 'x_foot'] = 2

        self.mpc.bounds["lower", "_u", "omega"] = -omega_max # in rad/sec
        self.mpc.bounds["upper", "_u", "omega"] = omega_max # in rad/sec
        
        self.mpc.bounds["lower", "_x", "theta"] = -2*casadi.pi # bounds should match the encoder calculations
        self.mpc.bounds["upper", "_x", "theta"] = 2*casadi.pi
        
        self.mpc.bounds["lower", "_x", "r0_x"] = self.X_MIN 
        self.mpc.bounds["upper", "_x", "r0_x"] = self.X_MAX 
        
        suppress_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())
        
    def start(self):
        
        # This loop rate is set by ROS and is at 200 Hz
        print("Starting control loop...")
        while not rospy.is_shutdown():
        
            if self.rad_flag == True:
             
                theta = self.motor2_rad
                r0_x = float(self.ball_pos.linear.x)
                r1_x = float(self.ball_pos.angular.x)
              
                u_opt = self.mpc.make_step(
                    np.array([theta, r0_x, r1_x,])
                )
                
                # --- EXTRACTION LOGIC ---
                # mpc.data is the do_mpc storage object. 
                # We take the last entry [-1] of the specific auxiliary variable name.
                # Replace 'collision_gain_X' with the specific name you are using (e.g., collision_gain_4)
                gain_name = f"collision_gain_{rospy.get_param('/x_solver_parameters/collision_gain_type')}"
                
                try:
                    # .data['_aux', gain_name] returns a history of the auxiliary variable
                    current_gain = self.mpc.data['_aux', gain_name][-1, 0]
                    current_dist = self.mpc.data['_aux', "dist"][-1, 0]
                    self.gain_pub.publish(float(current_gain))
                    self.dist_pub.publish(float(current_dist))
                except KeyError:
                    rospy.logwarn(f"Auxiliary variable {gain_name} not found in mpc.data")
                # -------------------------
                
                #print(u_opt.shape)
                self.rod_angular_vel = u_opt[0][0]
                self.omega_cmd.angular.x = self.rod_angular_vel # already in radians
                self.cmd_pub.publish(self.omega_cmd)

            self.rate.sleep() # QUESTION: The loop is running at 200 HZ but the solver is calculating based on a 1/60 dt. Does this cause problems?


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()

