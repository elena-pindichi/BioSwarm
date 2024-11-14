import numpy as np
import casadi as cas
import math
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

#CONSTANTS PART

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


# State-space model: x+ = Ax + Bu; y = Cx + Du
H = 0.2

A = np.block([[np.eye(2), H*np.eye(2)],
              [np.zeros((2,2)), np.eye(2)]])
B = np.vstack([np.zeros((2,2)), H*np.eye(2)])
C = np.hstack([np.eye(2), np.zeros((2,2))])
D = np.zeros((2,2))

# Model dimension
DX, DU = np.shape(B)
DY = np.shape(C)[0]

# Initial conditions
XO = np.array([0.0, 0.0, 0.0, 0.0])
U0 = np.zeros((DU))

# Goal 
XGOAL = np.array([1, 2, 0.0, 0.0])

# Constraints
UMIN = -1 * 0.15
UMAX = +1 * 0.15
D_U_MIN = -0.05
D_U_MAX = +0.05
YMIN = -10
YMAX = +10
#xmin = np.array([ymin, -0.2])
#xmax = np.array([ymax, 0.5])

# Define control parameters
# Weighting matrices 
Q = np.eye(DX) # cost for the state x
R = 1  # cost for the input u
Qy = np.eye(DY)  # cost for the output y
P = 10 * Q  # cost for the terminal cost
# Number of predictions and simulations
N_PRED = 5
N_SIM = 100


class test_ros_mpc(Node):
    def __init__(self):
        super().__init__('test_node')
        self.timer = self.create_timer(H, self.timer_callback)
        # self.last_pos_x = None
        # self.last_pos_y = None
        # self.last_vel_x = 0.0
        # self.last_vel_y = 0.0
        # self.last_vel_lin = 0.0
        # self.last_vel_ang = 0.0
        # self.last_acc_x = 0.0
        # self.last_acc_y = 0.0
        self.iteration = 0    
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
        self.x = self.solver.variable(DX, N_PRED + 1)
        self.u = self.solver.variable(DU, N_PRED)
        self.xinit = self.solver.parameter(DX, 1)
        self.uinit = self.solver.parameter(DU, 1)

        # Initialize constraints
        self.solver.subject_to(self.x[:, 0] == self.xinit) 

        for k in range(0, N_PRED):
            self.solver.subject_to(self.x[:, k + 1] == cas.mtimes(A, self.x[:, k]) + cas.mtimes(B, self.u[:, k]))  # dynamics

            # input magnitude constraints
            self.solver.subject_to(UMIN <= self.u[:, k])
            self.solver.subject_to(self.u[:, k] <= UMAX)

            # state magnitude constraints
            self.solver.subject_to(YMIN <= cas.mtimes(C, self.x[:, k]) + cas.mtimes(D, self.u[:, k]))
            self.solver.subject_to(cas.mtimes(C, self.x[:, k]) + cas.mtimes(D, self.u[:, k]) <= YMAX)
            
            # additional state constraints solver.subject_to(xmin[1] <= x[1, k] <= xmax[1])  

            if k == 0:
                self.solver.subject_to(D_U_MIN <= self.u[:, k] - self.uinit)
                self.solver.subject_to(self.u[:, k] - self.uinit <= D_U_MAX)
            else:
                self.solver.subject_to(D_U_MIN <= self.u[:, k] - self.u[:, k - 1])
                self.solver.subject_to(self.u[:, k] - self.u[:, k - 1] <= D_U_MAX)

        # Initialize objective
        self.objective = 0

        for k in range(0, N_PRED):
            if k == 0:
                self.objective += cas.mtimes(cas.mtimes(cas.transpose(self.x[:, k] - XGOAL), Q), self.x[:, k] - XGOAL) + cas.mtimes(cas.mtimes(cas.transpose(self.u[:, k] - U0), R), self.u[:, k] - U0)  # quadratic cost function
            else:
                self.objective += cas.mtimes(cas.mtimes(cas.transpose(self.x[:, k] - XGOAL), Q), self.x[:, k] - XGOAL) + cas.mtimes(cas.mtimes(cas.transpose(self.u[:, k] - self.u[:, k-1]), R), self.u[:, k] - self.u[:, k-1])  # quadratic cost function

        self.objective += cas.mtimes(cas.mtimes(cas.transpose(self.x[:, N_PRED] - XGOAL), P), self.x[:, N_PRED] - XGOAL)

        # Define the solver
        self.solver.minimize(self.objective)
        options = {'ipopt' : {'print_level': 0, 'sb': 'yes'}, 'print_time': 0}
        self.solver.solver('ipopt', options)

        # Create graph
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, N_SIM)  # Limite initiale de l'axe x
        self.ax.set_ylim(-1, 3)  # Limite initiale de l'axe y
        self.ax.set_title("Output y")
        self.ax.grid()
        self.ax.legend(['y1', 'y2'], loc='upper right')
        self.x_data = []
        self.y1_data = []
        self.y2_data = []
        plt.ion() 

        # Simulation variables
        self.usim = np.zeros((DU, 1))
        self.ysim = np.zeros((DY, 1))
        self.xsim = np.zeros((DX, 1))
        self.xsim[:, 0] = XO
        self.usim[:, 0] = U0

    def position_callback(self, msg):
        # Mettre à jour la dernière position reçue
        self.last_pos_x = msg.pose.pose.position.x
        self.last_pos_y = msg.pose.pose.position.y

    def timer_callback(self):
        if self.iteration < 100:
            self.solver.set_value(self.xinit, self.xsim[:, 0])
            self.solver.set_value(self.uinit, self.usim)

            sol = self.solver.solve()
            usol = sol.value(self.u)
            

            self.usim[:, 0] = usol[:, 0]
            self.ysim[:, 0] = C @ self.xsim[:, 0] + D @ self.usim[:, 0]  # update the dynamics
            self.xsim[:, 0] = A @ self.xsim[:, 0] + B @ self.usim[:, 0]  # update the dynamics
            
            self.x_data.append(self.iteration)
            self.y1_data.append(self.ysim[0, 0])
            self.y2_data.append(self.ysim[1, 0])

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
            self.twist.linear.x = math.sqrt(self.xsim[2, 0] **2 + self.xsim[3, 0] **2)
            self.twist.angular.z = math.atan2(self.xsim[3,0], self.xsim[2,0])
            self.pub.publish(self.twist)
            self.iteration += 1
            print(f"Position : x = {self.last_pos_x} y = {self.last_pos_y}")
        
        if self.iteration == 100:
            plt.ioff()
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
        # twist = Twist()
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = 0.0
        # node.pub.publish(twist)
        # node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

