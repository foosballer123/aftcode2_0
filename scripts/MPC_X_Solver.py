import do_mpc
from casadi import *
from casadi.tools import *

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
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

    def __init__(self, Y=360, P_D=112):

        rospy.init_node('mpc_solver', anonymous=True)

        rospy.Subscriber("/ball_pos", Twist, self.ball_callback, queue_size=10)
        #rospy.Subscriber("/motor1_pos", Float32, self.player_callback, queue_size=10)
        rospy.Subscriber("/motor2_rad", Float32, self.angular_callback, queue_size=10)
        self.cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size=10)

        self.rate = rospy.Rate(200)

        self.ball_pos = Twist()
        self.motor1_pos = 0
        self.motor2_rad = 0
        self.omega_cmd = Twist()
        self.ball_flag = False
        self.rad_flag = False
        
        self.P_D = P_D # distance between the players along the rod (used to calculate ball offset (r0))
        self.dt = 1/60  # should match the refresh rate of the camera
        self.Y = Y
        self.Y_MAX = 360
        self.Y_MIN = 0
        #self.rod_vel = 0
        self.rod_angular_vel = 0
        self.X_ROD = 40
        self.L = 50
        self.r_foot = 5
        self.r_ball = 15
        
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
        
    #def player_callback(self, msg):
    #    self.motor1_pos = self.pps * msg.data  # converting steps into pixels
              
    def angular_callback(self, msg):
        #offset = math.pi*0.3
        #rad = msg.data*((2*math.pi)/400) + offset # steps * rad/step = rad **CHECK THE CONTROLLER LOOP**
        self.motor2_rad = msg.data 
        if self.rad_flag == False:
            self.rad_flag = True
            
    def init_mpc(self):

        input_weight = 0.2
        resting_weight = 0 #200
        position_error_weight = 0 #25000
        goal_error_weight = 1000
        velocity_error_weight = 15
        prediction_steps = 6
        input_rate_weight = 0.2
        omega_max = 31 # in rad/sec

        model = do_mpc.model.Model('discrete')

        # for modeling the controlled variable
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        x_foot = model.set_variable(var_type='_x', var_name='x_foot', shape=(1, 1)) 
        
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
        
        #model.set_expression(
        #    expr_name="collision_gain", expr=casadi.tanh(50 * casadi.fmax(0, 20 - casadi.fabs(dist)))
        #)
        #collision_gain = model.aux["collision_gain"]
        
        model.set_expression(
            expr_name="collision_gain", expr= (x_foot-r0_x)/(640-x_foot)
        )
        collision_gain = model.aux["collision_gain"]
        
        ### Alternative expression ###
        #model.set_expression(
        #    expr_name="collision_gain", expr=( 1 / ((1/10)*casadi.fabs(x_foot - r0_x) - 1.1)**100 + 1 )
        #)
        #collision_gain = model.aux["collision_gain"]
        
        print("Collision gain", collision_gain)
        model.set_rhs("theta", theta + omega * self.dt)
        model.set_rhs("x_foot", self.X_ROD + self.L*casadi.sin(theta)) # -theta_foot ???
        
        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain)*2000) # 500 pix/sec clears field in ~500/640 second 
        model.set_rhs("r1_y", r1_y) 

        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * omega**2 + goal_error_weight * (collision_gain)**2 + resting_weight * (casadi.fmax(0, casadi.fabs(theta - ((15/8)*casadi.pi)) - 0.04))**2 # dead-zone / dead-band
        )
        model.set_expression(
            expr_name="meyer_term", expr= casadi.SX(0) + position_error_weight * (dist) ** 2# + goal_error_weight * (r0_x - 640)**2
        )
        
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
        
        self.mpc.bounds["lower", "_x", "theta"] = -2*casadi.pi # bounds should match the calculations by the encoder
        self.mpc.bounds["upper", "_x", "theta"] = 2*casadi.pi
        
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
                x_foot = self.X_ROD + self.L*math.sin(theta)
                
                r0_x = float(self.ball_pos.linear.x)
                r0_y = float(self.ball_pos.linear.y)
                r1_x = float(self.ball_pos.angular.x) # zero for testing
                r1_y = float(self.ball_pos.angular.y)
               
                print(theta)
                
                # Remember to pass proper parameters!!!
                u_opt = self.mpc.make_step(
                    np.array([theta, x_foot, r0_x, r0_y, r1_x, r1_y])
                )
                
                #print(u_opt.shape)
                self.rod_angular_vel = u_opt[0][0]
                self.omega_cmd.linear.y = self.rod_angular_vel # already in radians
                self.cmd_pub.publish(self.omega_cmd)

            self.rate.sleep() # QUESTION: The loop is running at 200 HZ but the solver is calculating based on a 1/60 dt. Does this cause problems?


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()

