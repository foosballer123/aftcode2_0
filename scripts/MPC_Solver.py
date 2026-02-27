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
#    - Find P_D (the distance between blue players along the same rod in pixels)
#
#    - to convert radians to pixels:
#    -     find how many rotations it takes to clear the field 
#    -     radians / Y of Camera Frame = radians / pixel -> rad/pix * pix/s = rad/sec
#    -     (525 steps / ~330 pix) * (2pi radians / 400 steps) =  0.025 rad/pix
#
#    - y = 0 is away from the hardware
#    - y = 360 is closest to the hardware

# TO-DO (after 1/21/2026):
#	- My conversions are off - it takes the linear stage 525 steps to clear ~1/3 of the y-distance of the field
#	- ~1/3 of the field in the y-direction is ~120 pixels -> 525 steps / ~120 pixels
#	- This might be throwing off the rest of my calculations!

# TO-DO: Find a way to embed the table boundaries into the MPC solver
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
        prediction_steps = 60
        input_rate_weight = 0.2
        v_max = 31 * self.ppr  # 31 rad/s * pix/rad = pix/s

        model = do_mpc.model.Model('discrete')

        # for modeling the controlled variable
        y_torso = model.set_variable(var_type='_x', var_name='y_torso', shape=(1, 1))
        theta_foot = model.set_variable(var_type='_x', var_name='theta_foot', shape=(1, 1))
        x_foot = model.set_variable(var_type='_x', var_name='x_foot', shape=(1, 1)) # calculate x_foot from theta using another variable type?
        # y_foot = y_torso
        
        y_vel = model.set_variable(var_type='_u', var_name='y_vel')
        omega_foot = model.set_variable(var_type='_u', var_name='omega_foot')
        
        # time-varying parameters (position + velocity of the ball)
        r0 = model.set_variable(var_type='_tvp', var_name='r0', shape=(2, 1))
        r1 = model.set_variable(var_type='_tvp', var_name='r1', shape=(2, 1))
        tau = model.set_variable(var_type='_tvp', var_name='tau')

        model.set_rhs("y_torso", y_vel * self.dt + y_torso)
        model.set_rhs("theta_foot", theta_foot + omega_foot * self.dt)
        model.set_rhs("x_foot", self.X_ROD + self.L*casadi.sin(-theta_foot)) # x_foot = X_ROD + L*math.sin(-theta_foot)
        
        y_ref_expr = r0[1] + r1[1] * tau
        #y_ref_expr_2 = r1
        
        model.set_expression('y_ref', y_ref_expr)
        #model.set_expression('y_ref_2', y_ref_expr_2)
        
        # THEORY
        # The cost function consists of two terms - a lagrange term (running cost) and a meyer term (terminal cost).
        # The running cost is associated with every step in the prediction horizon while the terminal cost is only associated with the last step.
        # Meaning, if you include wall collisions in your ball dynamics which cancel the ball velocity vector at some future state, than the solver could still prioritze following the ball based on the running cost.
        
        model.set_expression(
            expr_name="lagrange_term", expr=input_weight * y_vel ** 2
        )
        model.set_expression(
            expr_name="meyer_term", expr=position_error_weight * (model.aux['y_ref'] - y_torso) ** 2 + (r0[0] - 640)**2
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
        self.mpc.scaling['_x', 'y_torso'] = 2

        self.mpc.bounds["lower", "_u", "y_vel"] = -v_max
        self.mpc.bounds["upper", "_u", "y_vel"] = v_max
        self.mpc.bounds["lower", "_u", "omega_foot"] = -v_max
        self.mpc.bounds["upper", "_u", "omega_foot"] = v_max
        
        suppress_ipopt = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        self.mpc.set_param(nlpsol_opts=suppress_ipopt)

        tvp_template = self.mpc.get_tvp_template()
        
        # WARNING: There might be something wierd here with the ball coordinates when the offset is applied
        # TRY: Makes sure that the ball coordinate always stays within the table boundaries!
        # QUESTION: Does the solver account for the offset in the prediction horizon?
        
        # The tvp function takes the current time step and projects it across the prediction horizon according to its internal dynamics.
        # Meaning: If the dynamics are not included in the 'for k in range(self.mpc.settings.n_horizon + 1):' logic, they do not exist in the prediction horizon.
        # Meaning: The offset is only calculated as a function of the FIRST TIME STEP.
        # Meaning: Without any explicit table boundaries, the future positions of the ball would extend beyond them starting from the offset position.
        
        def tvp_fun(t_now):
            r0_now = casadi.DM([float(self.ball_pos.linear.x), float(self.ball_pos.linear.y)])
            r1_now = casadi.DM([float(self.ball_pos.angular.x), float(self.ball_pos.angular.y)]) # casadi.DM([0, 0]) 
            horizon = Twist()
            
            r0_x_k = 0
            r0_y_k = 0
            
            # Center player
            if (120 < r0_now[1] < 240):
                offset = self.P_D
            # Far left
            elif (self.Y_MIN < r0_now[1] <= 120):
                offset = 0
            # Far right
            elif (240 <= r0_now[1] < self.Y_MAX):
                offset = 2 * self.P_D
            else:
                offset = 0

            r0_offset = r0_now[1] - offset
            print("r0_offset variable type", type(r0_offset))
            self.r0_pub.publish(r0_offset) # publishing the ball position with offset to understand the tvp
            
            #print("[Pre] Variable types (player):", type(x_foot), type(y_torso))
            #print("[Pre] Variable types (ball):", type(r0_now[0]), type(r0_now[1]))
            dx = casadi.SX(r0_now[0]) - x_foot
            dy = r0_offset - y_torso
            #print("[Post] Variable types (d_):", type(dx), type(dy))
            dist = casadi.sqrt(dx**2 + dy**2)
            #print("[Post] Variable type (dist):", type(dist))
            print(dist)
            print("Equation", x_foot, " of type", type(x_foot))
            overlap = casadi.SX(casadi.fmax(0, (self.r_ball + self.r_foot) - dist)) # make sure this works the same as the laptop code
            collision_gain = casadi.SX(casadi.tanh(50 * overlap))
            
            m_b = 5
            m_f = 25
            u_b = r1_now[0]
            #u_f = self.L*omega_foot*casadi.cos(-theta_foot)
            u_f = 100
            horizon.angular.z = u_f # for testing the velocity of the foot will have to be simulated
            horizon.linear.z = collision_gain # FOR TESTING
            
            v_x = ( (m_b-m_f)*u_b + 2*m_f*u_f ) / ( m_f+m_b ) # do this calculation by hand
            horizon.angular.x = v_x
            #r1_new = [(1-collision_gain)*r1_now[0] + collision_gain*v_x, r1_now[1]]
            #if collision_gain > 0:
            #    print("Eureka!")
                
            r1_new = [v_x, r1_now[1]]
            
            for k in range(self.mpc.settings.n_horizon + 1):
                tvp_template['_tvp', k, 'r0'] = casadi.DM([r0_now[0], r0_offset])
                tvp_template['_tvp', k, 'r1'] = r1_new #r1_now
                tvp_template['_tvp', k, 'tau'] = k * self.dt
                
                # HYPOTHESIS: The solver handles tvp calculations outside of the function so to visualize them I have to repeat them here.
                # REASON: Was getting static position measurements but tested k by publishing to a topic and saw that it was incrementing.
                if k == int(self.mpc.settings.n_horizon):
                    r0_x_k = r0_now[0] + r1_new[0] * k * self.dt
                    r0_y_k = r0_now[1] + r1_new[1] * k * self.dt
                
                horizon.linear.x = r0_x_k #k #casadi.SX(r0_now[0])
                horizon.linear.y = r0_y_k #k #casadi.SX(r0_offset)
                self.r0_horizon_pub.publish(horizon)
                
            return tvp_template


        self.mpc.set_tvp_fun(tvp_fun)
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
        
            if self.ball_flag == True:
                #print("Retrieving optimal control values.")
                u_opt = self.mpc.make_step(
                    np.array([int(self.motor1_pos), int(self.motor2_pos), int(self.X_ROD + self.L*math.sin(-self.motor2_pos))])
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

