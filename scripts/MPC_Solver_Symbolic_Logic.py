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
        rospy.Subscriber("/motor1_pos", Float32, self.player_callback, queue_size=10)
        rospy.Subscriber("/motor2_pos", Float32, self.angular_callback, queue_size=10)
        self.torso_pub = rospy.Publisher('/mpc_torso_position', Float32, queue_size=10)
        self.cmd_pub = rospy.Publisher('/omega_d', Twist, queue_size=10)
        self.r0_pub = rospy.Publisher('/r0_tvp', Float32, queue_size=10)
        self.r0_horizon_pub = rospy.Publisher('/r0_horizon', Twist, queue_size=60)
        
        self.rate = rospy.Rate(200)

        self.ball_pos = Twist()
        self.motor1_pos = 0
        self.motor2_pos = 0
        self.omega_cmd = Twist()
        self.ball_flag = False
        
        self.P_D = P_D # distance between the players along the rod (used to calculate ball offset (r0))
        self.dt = 1/60  # should match the refresh rate of the camera
        self.Y = Y
        self.Y_MAX = 360
        self.Y_MIN = 0
        self.rod_vel = 0
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
        
    def player_callback(self, msg):
        self.motor1_pos = self.pps * msg.data  # converting steps into pixels
        
        # Applying an offset inside the callback so that the players match the ball offset
        # Center player
        #if (120 < self.ball_pos.linear.y < 240):
        #    offset = 120 # self.P_D
        # Far left
        #elif (self.Y_MIN < self.ball_pos.linear.y <= 120):
        #    offset = 0
        # Far right
        #elif (240 <= self.ball_pos.linear.y < self.Y_MAX):
        #    offset = 2 * 120 # self.P_D
        #else:
        #    offset = 0
        
    def angular_callback(self, msg):
        offset = math.pi*0.3
        rad = msg.data*((2*math.pi)/400) + offset # steps * rad/step = rad
        self.motor2_pos = rad #int(self.X_ROD + self.L*math.sin(-rad))
                
    def init_mpc(self):

        input_weight = 0
        position_error_weight = 50
        velocity_error_weight = 15
        prediction_steps = 6
        input_rate_weight = 0.2 # 0.2
        v_max = 31 * self.ppr  # 31 rad/s * pix/rad = pix/s

        model = do_mpc.model.Model('discrete')

        # for modeling the controlled variable
        y1 = model.set_variable(var_type='_x', var_name='y1', shape=(1, 1))
        y2 = model.set_variable(var_type='_x', var_name='y2', shape=(1, 1))
        y3 = model.set_variable(var_type='_x', var_name='y3', shape=(1, 1))
        theta = model.set_variable(var_type='_x', var_name='theta', shape=(1, 1))
        x_foot = model.set_variable(var_type='_x', var_name='x_foot', shape=(1, 1)) # calculate x_foot from theta using another variable type?
        # y_foot = y_torso
        
        # make the ball a state variable 
        r0_x = model.set_variable(var_type='_x', var_name='r0_x', shape=(1, 1))
        r0_y = model.set_variable(var_type='_x', var_name='r0_y', shape=(1, 1))
        r1_x = model.set_variable(var_type='_x', var_name='r1_x', shape=(1, 1))
        r1_y = model.set_variable(var_type='_x', var_name='r1_y', shape=(1, 1))
        
        y_vel = model.set_variable(var_type='_u', var_name='y_vel')
        omega = model.set_variable(var_type='_u', var_name='omega')
        
        model.set_expression(expr_name="z1", expr=casadi.tanh(50*casadi.fmax(0, r0_y))
        )
        z1 = model.aux["z1"]
        model.set_expression(expr_name="z2", expr=casadi.tanh(50*casadi.fmax(0, r0_y-120))
        ) 
        z2 = model.aux["z2"]
        model.set_expression(expr_name="z3", expr=casadi.tanh(50*casadi.fmax(0, r0_y-240))
        )
        z3 = model.aux["z3"]
        
        #model.set_expression(
        #    expr_name="dist", expr=casadi.sqrt( (x_foot - r0_x)**2 + ((z1*(1-z2)*(1-z3)*y1 + z2*(1-z3)*y2 + z3*y3) - r0_y)**2 ) #casadi.fmin((y1 - r0[1])**2, casadi.fmin((y2 - r0[1])**2, (y3 - r0[1])**2))
        #)
        #dist = model.aux["dist"]
        
        model.set_expression(
            expr_name="y_ref", expr=((z1*(1-z2)*(1-z3)*y1 + z2*(1-z3)*y2 + z3*y3) - r0_y)
        )
        y_ref = model.aux["y_ref"]
        
        model.set_expression(
            expr_name="collision_gain", expr=casadi.tanh(50 * casadi.fmax(0, 20 - casadi.fabs(x_foot - r0_x)))
        )
        collision_gain = model.aux["collision_gain"]
        
        #model.set_expression(
        #    expr_name="collision_gain", expr=( 1 / ((1/10)*casadi.fabs(x_foot - r0_x) - 1.1)**100 + 1 )
        #)
        #collision_gain = model.aux["collision_gain"]
        
        print("Collision gain", collision_gain)
        model.set_rhs("y1", y1 + y_vel * self.dt)
        model.set_rhs("y2", y1 + self.P_D)
        model.set_rhs("y3", y1 + 2*self.P_D)
        model.set_rhs("theta", theta + omega * self.dt)
        model.set_rhs("x_foot", self.X_ROD + self.L*casadi.sin(theta)) # x_foot = X_ROD + L*math.sin(-theta_foot)
        model.set_rhs("r0_x", r0_x + r1_x * self.dt)
        model.set_rhs("r0_y", r0_y + r1_y * self.dt)
        model.set_rhs("r1_x", (1-collision_gain)*r1_x + (collision_gain)*100) 
        model.set_rhs("r1_y", r1_y) 

        #model.set_expression(
        #    expr_name="collision_gain", expr=casadi.tanh(50 * casadi.fmax(0, 30 - dist))
        #)
        #collision_gain = model.aux["collision_gain"]
        
        #model.set_expression(
        #    expr_name="lagrange_term", expr=input_weight * (y_vel**2 + omega**2) + 50*theta**2 # 10000000000*((theta) - (casadi.pi))**2 #- 10*r1_x #+ 50*(y1 - 0)**2 #((y1 - 60)**2 + (y2 - 180)**2 + (y3 - 240)**2) 
        #)
        #model.set_expression(
        #    expr_name="meyer_term", expr=position_error_weight * (y_ref) ** 2 # + 50*(r0_x - 640)**2
        #)
        
        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * (y_vel**2) 
        )
        model.set_expression(
            expr_name="meyer_term", expr=position_error_weight * (y_ref) ** 2 
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
        self.mpc.set_rterm(omega=0.2)
        #self.mpc.scaling['_x', 'y_torso'] = 2

        self.mpc.bounds["lower", "_u", "y_vel"] = -v_max # in pix/sec
        self.mpc.bounds["upper", "_u", "y_vel"] = v_max # in pix/sec
        self.mpc.bounds["lower", "_u", "omega"] = -20 # in rad/sec
        self.mpc.bounds["upper", "_u", "omega"] = 20 # in rad/sec
        #self.mpc.bounds["lower", "_x", "y1"] = 0
        #self.mpc.bounds["upper", "_x", "y1"] = 120
        #self.mpc.bounds["lower", "_x", "y2"] = 120
        #self.mpc.bounds["upper", "_x", "y2"] = 240
        #self.mpc.bounds["lower", "_x", "y3"] = 240
        #self.mpc.bounds["upper", "_x", "y3"] = 360
        
        suppress_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        self.mpc.setup()
        print("MPC solver defined!")

        self.mpc.x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.mpc.set_initial_guess()
        print("Initial state variables:", model.x.labels())
        print("Initial input variables:", model.u.labels())
        
    def start(self):
        
        # This loop rate is set by ROS and is at 200 Hz
        print("Starting control loop...")
        while not rospy.is_shutdown():
        
            if self.ball_flag == True:
                #print("Retrieving optimal control values.")
                
                y1 = int(self.motor1_pos)
                y2 = y1 + self.P_D
                y3 = y1 + 2*self.P_D
                theta = -self.motor2_pos
                x_foot = int(self.X_ROD + self.L*math.sin(-self.motor2_pos))
                r0_x = int(self.ball_pos.linear.x)
                r0_y = int(self.ball_pos.linear.y)
                r1_x = float(self.ball_pos.angular.x)
                r1_y = float(self.ball_pos.angular.y)
               
                print(theta)
                # Remember to pass parameters!!!
                u_opt = self.mpc.make_step(
                    np.array([y1, y2, y3, theta, x_foot, r0_x, r0_y, r1_x, r1_y])
                )
                #print(u_opt.shape)
                self.rod_vel = u_opt[0][0]
                self.rod_angular_vel = u_opt[1][0]
                #print(self.mpc.data['_x', 'y_torso'][0])
                #self.torso_pub.publish(float(self.mpc.data['_x', 'y_torso'][0]))
                self.omega_cmd.linear.x = self.rod_vel * self.rpp
                self.omega_cmd.linear.y = self.rod_angular_vel # already in radians
                self.cmd_pub.publish(self.omega_cmd)

            self.rate.sleep() # QUESTION: The loop is running at 200 HZ but the solver is calculating based on a 1/60 dt. Does this cause problems?


if __name__ == '__main__':
    fooBot = MPC_Solver()
    fooBot.init_mpc()
    fooBot.start()

