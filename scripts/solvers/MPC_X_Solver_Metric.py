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
        self.cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size=10)
        
        self.rate = rospy.Rate(200)
       
        self.dt = rospy.get_param('/x_solver_parameters/timestep')  # should match the refresh rate of the camera
        
        self.ball_flag = False
        self.rad_flag = False
        self.ball_pos = Twist()
        #self.motor1_pos = Float64MultiArray()
        self.motor2_rad = Float64MultiArray()
        
        self.omega_cmd = Twist()
        #self.rod_vel = 0
        self.rod_angular_vel = 0
        
        self.P_D = rospy.get_param('/table_measurements/distance_between_players') # distance between the players along the rod (even zones)
        self.Z_D = rospy.get_param('/table_measurements/distance_to_clear_zone')
        self.W_D = rospy.get_param('/table_measurements/player_distance_from_wall')
        self.zone_padding = 0.05
        
        self.Y = rospy.get_param('/table_measurements/field_height')
        self.Y_MAX = self.Y
        self.Y_MIN = 0
        
        self.X = rospy.get_param('/table_measurements/field_width')
        self.X_MAX = self.X
        self.X_MIN = 0
        self.X_ROD = rospy.get_param('/table_measurements/blue_rod_positions/rod_one')
        
        self.L_P = rospy.get_param('/table_measurements/player_length')
        self.R_F = rospy.get_param('/table_measurements/approximate_player_foot_diameter')
        self.R_B = rospy.get_param('/table_measurements/ball_diameter')
        
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
        
    #def player_callback(self, msg):
    #    self.motor1_pos = self.pps * msg.data  # converting steps into pixels
              
    def angular_callback(self, msg):
        #offset = math.pi*0.3
        self.motor2_rad = msg.data[0] 
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

        # for modeling the controlled variable
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        x_foot = model.set_variable(var_type='_x', var_name='x_foot', shape=(1, 1)) 
        #z_foot = ???
        
        # make the ball a state variable 
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        r0_y = model.set_variable(var_type='_x', var_name='r0_y', shape=(1, 1))
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))
        r1_y = model.set_variable(var_type='_x', var_name='r1_y', shape=(1, 1))
        
        omega = model.set_variable(var_type='_u', var_name='omega')
        
        model.set_expression(
            expr_name="dist", expr=(x_foot - r0_x)
        )
        dist = model.aux["dist"]
        
        if collision_gain_type == 1:
            model.set_expression(
                expr_name="collision_gain_1", expr=casadi.tanh(50 * casadi.fmax(0, 20 - casadi.fabs(dist)))
            )
            collision_gain = model.aux["collision_gain_1"]
            
        elif collision_gain_type == 2:
            model.set_expression(
                expr_name="collision_gain_2", expr= (x_foot-r0_x)/(self.X_MAX-x_foot)
            )
            collision_gain = model.aux["collision_gain_2"]
        
        elif collision_gain_type == 3:
            model.set_expression(
                expr_name="collision_gain_3", expr=( 1 / ((1/10)*casadi.fabs(x_foot - r0_x) - 1.1)**100 + 1 )
            )
            collision_gain = model.aux["collision_gain_3"]
        
        print("Collision Gain Equation (Type "+str(collision_gain_type)+"): ", collision_gain, "\n")
        
        model.set_rhs("theta", theta + omega * self.dt)
        model.set_rhs("x_foot", self.X_ROD + self.L_P*casadi.sin(theta)) 
        #z_foot = ???
        
        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain)*2000)
        model.set_rhs("r1_y", r1_y) 

        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * omega**2 + goal_error_weight * (collision_gain)**2 + resting_weight * (casadi.fmax(0, casadi.fabs(theta - (-(15/8)*casadi.pi)) - 0.04))**2 # dead-zone / dead-band
        )
        model.set_expression(
            expr_name="meyer_term", expr= casadi.SX(0) + position_error_weight * (dist) ** 2 # + goal_error_weight * (r0_x - 640)**2
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
        self.mpc.bounds["lower", "_x", "r0_y"] = self.Y_MIN 
        self.mpc.bounds["upper", "_x", "r0_y"] = self.Y_MAX
        
        suppress_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0, 0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())
        
    def start(self):
        
        # This loop rate is set by ROS and is at 200 Hz
        print("Starting control loop...")
        while not rospy.is_shutdown():
        
            if self.rad_flag == True:
             
                theta = self.motor2_rad
                x_foot = self.X_ROD + self.L_P*math.sin(theta)
                
                # MAKE SURE TO UPDATE THESE VARIABLES IN THE BALL DETECTION PIPELINE AND CONVERT TO METRIC
                r0_x = float(self.ball_pos.linear.x)
                r0_y = float(self.ball_pos.linear.y)
                r1_x = float(self.ball_pos.angular.x) # zero for testing
                r1_y = float(self.ball_pos.angular.y)
               
                #print(theta)
                
                # Remember to pass proper parameters!!!
                u_opt = self.mpc.make_step(
                    np.array([theta, x_foot, r0_x, r0_y, r1_x, r1_y])
                )
                
                #print(u_opt.shape)
                self.rod_angular_vel = u_opt[0][0]
                self.omega_cmd.angular.x = self.rod_angular_vel # already in radians
                self.cmd_pub.publish(self.omega_cmd)

            self.rate.sleep() # QUESTION: The loop is running at 200 HZ but the solver is calculating based on a 1/60 dt. Does this cause problems?


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()

