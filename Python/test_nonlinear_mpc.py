import numpy as np
import casadi as cas
import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion

# Constants

#  Parameters
NPRED = 25     # Prediction horizon
NSIM = 250
TE = 0.1
Q_VAL= 10000         # State weight value
R_VAL = 1           # Input weight value
P_VAL = 1        # Terminal state value

# Physical parameters
RADIUS = 33e-3;           # Radius of wheels
L = 160e-3;          # Distance between wheels

#  Define system dimensions
DX = 3;              # State dimensions: x, y, theta
DU = 2;              # Control dimensions: omega_r (right wheel), omega_l (left wheel)

#  Initial condition
XREF = np.array([2, 3, 0])    # Reference state [x_ref; y_ref; theta_ref]

# Constraints on wheel angular velocities
OMEGA_R_MIN = -10
OMEGA_R_MAX = 10   # Right wheel angular velocity limits
OMEGA_L_MIN = -10
OMEGA_L_MAX = 10   # Left wheel angular velocity limits
XMIN = -1 * 0.1 *  np.array([1.0 ,1.0, 1.0])#np.ones(3)          # State limits
XMAX =  1 * 0.1 * np.array([1.0 ,1.0, 1.0]) #np.ones(3) 

# Weights for cost function
Q = np.vstack([[10000, 0, 0], [0, 10000, 0], [0, 0, 100]])#Q_VAL * np.eye(DX)
R = R_VAL * np.eye(DU)
P = P_VAL * Q

# Input conversion
IN_CONV = np.vstack([[RADIUS/2, RADIUS/2], [RADIUS/L, -RADIUS/L]])

class test_ros_mpc(Node):
    def __init__(self):
        super().__init__('test_node')
        self.timer = self.create_timer(TE, self.timer_callback)
        self.iteration = 0  
        self.last_pos_x = None
        self.last_pos_y = None 
        self.last_yaw = None 
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        #Position subscription
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.position_callback,
            10
        )

        #Velocity publisher
        self.pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            QoSProfile(depth=10)
        )

        # Optimization problem using CasADi
        self.solver = cas.Opti()  # create an Opti object

        # Define variables
        self.x = self.solver.variable(DX, NPRED + 1)
        self.u = self.solver.variable(DU, NPRED)
        self.xinit = self.solver.parameter(DX, 1)

        #  Nonlinear dynamics of the unicycle model with omega_r and omega_l
        f_dynamics = cas.vertcat(
            (RADIUS / 2) * (self.u[0] + self.u[1]) * cas.cos(self.x[2]),
            (RADIUS / 2) * (self.u[0] + self.u[1]) * cas.sin(self.x[2]),
            (RADIUS / L) * (self.u[0] - self.u[1])
        )
        self.f_dynamics_func = cas.Function('f_dynamics', [self.x, self.u], [f_dynamics]) # Wrap it as a CasADi function
        

        # Initialize constraints
        self.solver.subject_to(self.x[:, 0] == self.xinit)
        for k in range(0, NPRED) : 
            # State update constraint using discretized nonlinear dynamics
            self.solver.subject_to(self.x[:, k+1] == (self.x[:, k] + TE * self.f_dynamics_func(self.x[:, k], self.u[:, k])))
            self.solver.subject_to(XMIN <= (self.x[:, k + 1] - self.x[:, k]))
            self.solver.subject_to((self.x[:, k + 1] - self.x[:, k]) <= XMAX)
            # % solver.subject_to(-H * x(1:2, k+1) <= -b + M * alpha(:, k+1))
            # % solver.subject_to(sum(alpha(:, k+1)) <= 3)
            
            #  Control input constraints
            self.solver.subject_to(OMEGA_R_MIN <= self.u[0, k])
            self.solver.subject_to(self.u[0, k] <= OMEGA_R_MAX)
            self.solver.subject_to(OMEGA_L_MIN <= self.u[1, k])
            self.solver.subject_to(self.u[1, k] <= OMEGA_L_MAX)

        # Define the objective function
        objective = 0
        for k in range(0, NPRED):
            #  Stage cost (penalty for deviation from reference)
            # objective += cas.times(cas.times(cas.transpose(x[:, k] - XREF), Q), x[:, k] - XREF) + cas.times(cas.times(cas.transpose(u[:, k]), R), u[:, k])
            objective += cas.mtimes([cas.transpose(self.x[:, k] - XREF), Q, self.x[:, k] - XREF]) + cas.mtimes([cas.transpose(self.u[:, k]), R, self.u[:, k]])
        #  Terminal cost
        objective += cas.mtimes([cas.transpose(self.x[:, NPRED] - XREF), Q, self.x[:, NPRED] - XREF])
        # objective += cas.times(cas.times(cas.transpose(x[:, NPRED] - XREF), P), x[:, NPRED] - XREF)
        self.solver.minimize(objective)

        # Solver options 
        options = {'ipopt' : {'print_level': 0, 'sb': 'yes', 'tol' : 1e-6}, 'print_time': 0}
        self.solver.solver('ipopt', options)

        # Create graph
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, NSIM)  # Limite initiale de l'axe x
        self.ax.set_ylim(-1, 10)  # Limite initiale de l'axe y
        self.ax.set_title("Output y")
        self.ax.grid()
        self.ax.legend(['y1', 'y2'], loc='upper right')
        self.x_data = []
        self.y1_data = []
        self.y2_data = []
        plt.ion() 

        # Simulation variables
        self.usim = np.zeros((DU, 1))
        self.xsim = np.zeros((DX, 1))


    def position_callback(self, msg):
        # Mettre à jour la dernière position reçue
        self.last_pos_x = msg.pose.pose.position.x
        self.last_pos_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.last_yaw = yaw

    def timer_callback(self):
        if self.last_pos_x != None : 
            if self.iteration < NSIM:
                self.xsim[0, 0] = self.last_pos_x
                self.xsim[1, 0] = self.last_pos_y
                self.xsim[2, 0] = self.last_yaw
                self.solver.set_value(self.xinit, self.xsim[:, 0])

                sol = self.solver.solve()
                usol = sol.value(self.u)
                

                self.usim[:, 0] = usol[:, 0]
                self.xsim[:, 0] = self.xsim[:, 0] + TE * self.f_dynamics_func(self.xsim[:, 0], self.usim[:, 0]).full().flatten()
                
                self.x_data.append(self.iteration)
                self.y1_data.append(self.xsim[0, 0])
                self.y2_data.append(self.xsim[1, 0])

                if self.iteration > 0 :
                    self.stemlines1.remove()  # Supprime la version précédente pour éviter l'empilement
                    self.stemlines2.remove()
                
                self.stemlines1 = self.ax.stem(self.x_data, self.y1_data, linefmt='b', markerfmt='bo', basefmt=" ")
                self.stemlines2 = self.ax.stem(self.x_data, self.y2_data, linefmt='r', markerfmt='ro', basefmt=" ")
                if self.iteration == 0:
                    plt.show() 
                    print("Simulation started")
                self.fig.canvas.draw() 
                self.fig.canvas.flush_events()
                input = cas.mtimes(IN_CONV, self.usim[:, 0]).full()
                self.twist.linear.x = input[0, 0]
                self.twist.angular.z = input[1, 0]
                self.pub.publish(self.twist)
                self.iteration += 1
                print(f"Position : x = {self.last_pos_x} y = {self.last_pos_y}")
            
            if self.iteration == NSIM:
                plt.ioff()
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                print("Simulation ended")
                self.iteration += 1
            



        # if (self.last_pos_x != None) and (self.last_pos_y != None):
            # x = np.vstack([self.last_pos_x, self.last_pos_y, self.last_vel_x, self.last_vel_y])
            # self.solver.set_value(self.xinit, x)
            # self.solver.set_value(self.uinit, np.vstack([self.last_acc_x, self.last_acc_y]))               
            # sol = self.solver.solve()
            # usol = sol.value(self.u)
            # self.last_acc_x = usol[0,0]
            # self.last_acc_y = usol[1,0]  #Big problem 
            # x = A @ x + B @ usol[:, 0]

            # target_linear_velocity = check_linear_limit_velocity(math.sqrt(x[2,0] **2 + x[3,0] **2))
            # self.last_vel_lin = make_simple_profile(
            #     self.last_vel_lin,
            #     target_linear_velocity,
            #     (LIN_VEL_STEP_SIZE / 2.0))
            
            # target_angular_velocity = check_angular_limit_velocity(math.atan2(x[3,0], x[2,0]))
            # self.last_vel_ang = make_simple_profile(
            #     self.last_vel_ang ,
            #     target_angular_velocity,
            #     (ANG_VEL_STEP_SIZE / 2.0))

            # self.last_vel_x = self.last_vel_lin * math.cos(self.last_vel_ang)
            # self.last_vel_y = self.last_vel_lin * math.sin(self.last_vel_ang)

            # twist = Twist()
            # twist.linear.x = self.last_vel_lin
            # twist.linear.y = 0.0
            # twist.linear.z = 0.0
            # twist.angular.x = 0.0
            # twist.angular.y = 0.0
            # twist.angular.z = self.last_vel_ang
            # self.pub.publish(twist)

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)           

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output 

def main(args=None):
    rclpy.init(args=args)
    node = test_ros_mpc()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        #twist = Twist()
        #twist.linear.x = 0.0
        #twist.angular.z = 0.0
        #node.pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

