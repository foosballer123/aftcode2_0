import do_mpc
from casadi import *
from casadi.tools import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import rospy
import numpy as np
import math
import argparse

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

    def __init__(self, Y=360, P_D=112):

        rospy.init_node('mpc_solver', anonymous=True)

        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        rospy.Subscriber("/motor1_pos", Float32, self.player_callback, queue_size=10)
        #rospy.Subscriber("/motor2_pos", Float32, self.angular_callback, queue_size=10)
        self.cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size=10)

        self.rate = rospy.Rate(200)

        self.ball_pos = Twist()
        self.motor1_pos = 0
        #self.motor2_pos = 0
        self.omega_cmd = Twist()
        self.ball_flag = False
        
        self.P_D = P_D # distance between the players along the rod (used to calculate ball offset (r0))
        self.dt = 1/60  # should match the refresh rate of the camera
        self.Y = Y
        self.Y_MAX = 360
        self.Y_MIN = 0
        self.rod_vel = 0
        #self.rod_angular_vel = 0
        self.X_ROD = 40
        #self.L = 50
        #self.r_foot = 5
        #self.r_ball = 15
        
        self.pps = 120 / 525  # pixels per step (was 330/525 ... testing with 120/525)
        self.rpp = math.pow(self.pps,-1) * (2*math.pi / 400)	# radians per pixel (was 0.025 ... testing calculation)
        self.ppr = math.pow(self.rpp,-1)	# pixels per radian (was 40 ... testing calculation)

    def ball_callback(self, msg):
        """
        Callback function for receiving ball position data.
        Updates the variable ball_pos.
        """
        if self.ball_flag == False:
            self.ball_flag = True
        self.ball_pos = msg
        
    def player_callback(self, msg):
        self.motor1_pos = msg.data  # converting steps into pixels (self.pps *) 
              
    #def angular_callback(self, msg):
    #    offset = math.pi*0.3
    #    rad = msg.data*((2*math.pi)/400) + offset # steps * rad/step = rad
    #    self.motor2_pos = rad #int(self.X_ROD + self.L*math.sin(-rad))
                
    def init_mpc(self, input_weight, position_error_weight, velocity_error_weight, prediction_steps, input_rate_weight):

        input_weight = input_weight
        position_error_weight = position_error_weight
        velocity_error_weight = velocity_error_weight
        prediction_steps = prediction_steps
        input_rate_weight = input_rate_weight
        print("Solver Parameters")
        print("-----------------------------")
        print("Input Weight:", input_weight)
        print("Position Error Weight:", position_error_weight)
        print("Velocity Error Weight:", velocity_error_weight)
        print("Prediction Horizon:", prediction_steps)
        print("Input Rate Weight:", input_rate_weight,"\n") 
        v_max = 31 * self.ppr  # 31 rad/s * pix/rad = pix/s

        model = do_mpc.model.Model('discrete')

        # for modeling the controlled variable
        y1 = model.set_variable(var_type='_x', var_name='y1', shape=(1, 1))
        y2 = model.set_variable(var_type='_x', var_name='y2', shape=(1, 1))
        y3 = model.set_variable(var_type='_x', var_name='y3', shape=(1, 1))
     
        # make the ball a state variable 
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        r0_y = model.set_variable(var_type='_x', var_name='r0_y', shape=(1, 1))
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))
        r1_y = model.set_variable(var_type='_x', var_name='r1_y', shape=(1, 1))
        
        y_vel = model.set_variable(var_type='_u', var_name='y_vel')
       
        model.set_expression(expr_name="z1", expr=casadi.tanh(50*casadi.fmax(0, r0_y))
        )
        z1 = model.aux["z1"]
        model.set_expression(expr_name="z2", expr=casadi.tanh(50*casadi.fmax(0, r0_y-120))
        ) 
        z2 = model.aux["z2"]
        model.set_expression(expr_name="z3", expr=casadi.tanh(50*casadi.fmax(0, r0_y-240))
        )
        z3 = model.aux["z3"]
        
        model.set_expression(
            expr_name="y_error", expr=((z1*(1-z2)*(1-z3)*y1 + z2*(1-z3)*y2 + z3*y3) - r0_y)
        )
        y_error = model.aux["y_error"]
        
        model.set_rhs("y1", y1 + y_vel * self.dt)
        model.set_rhs("y2", y1 + self.P_D)
        model.set_rhs("y3", y1 + 2*self.P_D)

        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        model.set_rhs("r1_x", r1_x) 
        model.set_rhs("r1_y", r1_y) 

        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * (y_vel**2) 
        )
        model.set_expression(
            expr_name="meyer_term", expr=position_error_weight * (y_error) ** 2 
        )
        model.setup()
        print("Model defined!")
        
        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.settings.n_horizon = prediction_steps
        self.mpc.settings.t_step = self.dt

        lterm = model.aux["lagrange_term"]
        mterm = model.aux["meyer_term"]
        self.mpc.set_objective(lterm=lterm, mterm=mterm)

        self.mpc.set_rterm(y_vel=input_rate_weight)
        #self.mpc.scaling['_x', 'y_torso'] = 2

        self.mpc.bounds["lower", "_u", "y_vel"] = -v_max # in pix/sec
        self.mpc.bounds["upper", "_u", "y_vel"] = v_max # in pix/sec

        self.mpc.bounds["lower", "_x", "y1"] = 0
        self.mpc.bounds["upper", "_x", "y1"] = 120
        self.mpc.bounds["lower", "_x", "y2"] = 120
        self.mpc.bounds["upper", "_x", "y2"] = 240
        self.mpc.bounds["lower", "_x", "y3"] = 240
        self.mpc.bounds["upper", "_x", "y3"] = 360
        
        self.mpc.bounds["lower", "_x", "r0_x"] = 0
        self.mpc.bounds["upper", "_x", "r0_x"] = 640
        self.mpc.bounds["lower", "_x", "r0_y"] = 0
        self.mpc.bounds["upper", "_x", "r0_y"] = 360
        
        suppress_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0, 0, 0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())
        
    def start(self):
        
        # This loop rate is set by ROS and is at 200 Hz
        print("Starting control loop...")
        while not rospy.is_shutdown():
        
            if self.ball_flag == True:
                
                y1 = int(self.motor1_pos)
                y2 = y1 + self.P_D
                y3 = y1 + 2*self.P_D
                
                r0_x = float(self.ball_pos.linear.x)
                r0_y = float(self.ball_pos.linear.y)
                r1_x = float(self.ball_pos.angular.x)
                r1_y = float(self.ball_pos.angular.y)
               
                # Remember to pass proper parameters!!!
                u_opt = self.mpc.make_step(
                    np.array([y1, y2, y3, r0_x, r0_y, r1_x, r1_y])
                )
                
                #print(u_opt.shape)
                self.rod_vel = u_opt[0][0]
                self.omega_cmd.linear.x = self.rod_vel * self.rpp
                self.cmd_pub.publish(self.omega_cmd)

            self.rate.sleep() # QUESTION: The loop is running at 200 HZ but the solver is calculating based on a 1/60 dt. Does this cause problems?


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
    description="Script that initializes the MPC Y Solver with a defined set of parameters."
    )
    parser.add_argument("--input_weight", required=False, type=float, default=0)
    parser.add_argument("--position_error_weight", required=False, type=float, default=50)
    parser.add_argument("--velocity_error_weight", required=False, type=float, default=15)
    parser.add_argument("--prediction_steps", required=False, type=int, default=6)
    parser.add_argument("--input_rate_weight", required=False, type=float, default=0.2)
    args = parser.parse_args()
    
    a = args.input_weight
    b = args.position_error_weight
    c = args.velocity_error_weight
    d = args.prediction_steps
    e = args.input_rate_weight
	
    fooBot = MPC_Solver()
    fooBot.init_mpc(a, b, c, d, e)
    fooBot.start()

