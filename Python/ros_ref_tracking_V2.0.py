import numpy as np
import casadi as cas
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
NSIM = 300
TE = 0.1
Q_VAL= 100       # State weight value
R_VAL = 1000           # Input weight value
P_VAL = 1000       # Terminal state value

# Physical parameters
RADIUS = 33e-3;           # Radius of wheels
L = 160e-3;          # Distance between wheels

#  Define system dimensions
DX = 3;              # State dimensions: x, y, theta
DU = 2;              # Control dimensions: omega_r (right wheel), omega_l (left wheel)


T = np.linspace(0, 2 * np.pi, NSIM + NPRED + 1)
XREF = np.vstack((0.75*np.sin(T), 0.75*np.cos(T), np.zeros_like(T)))




#XREF = np.array([1.0, 1.0, 0])    # Reference state [x_ref; y_ref; theta_ref]

# Constraints on wheel angular velocities
V_MIN = -0.22
V_MAX = 0.22   # Right wheel angular velocity limits
OMEGA_MIN = -0.8
OMEGA_MAX = 0.8  # Left wheel angular velocity limits
XMIN = -1 * 0.2 *  np.array([1.0 ,1.0, 1.0])#np.ones(3)          # State limits
XMAX =  1 * 0.2 * np.array([1.0 ,1.0, 1.0]) #np.ones(3) 

# Weights for cost function
Q = Q = np.diag([Q_VAL, Q_VAL, 0.0001])      
R = R_VAL * np.eye(DU)
P = P_VAL * Q

def f_dynamics(x, u):
    return cas.vertcat(u[0] * cas.cos(x[2]),
                      u[0] * cas.sin(x[2]),
                      u[1])

class ros_ref_tracking(Node):
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

        # Initialize constraints
        self.solver.subject_to(self.x[:, 0] == self.xinit)
        for k in range(0, NPRED) : 
            # State update constraint using discretized nonlinear dynamics
            self.solver.subject_to(self.x[:, k+1] == (self.x[:, k] + TE * f_dynamics(self.x[:, k], self.u[:, k])))
            self.solver.subject_to(XMIN <= (self.x[:, k + 1] - self.x[:, k]))
            self.solver.subject_to((self.x[:, k + 1] - self.x[:, k]) <= XMAX)
            # % solver.subject_to(-H * x(1:2, k+1) <= -b + M * alpha(:, k+1))
            # % solver.subject_to(sum(alpha(:, k+1)) <= 3)
            
            #  Control input constraints
            self.solver.subject_to(V_MIN <= self.u[0, k])
            self.solver.subject_to(self.u[0, k] <= V_MAX)
            self.solver.subject_to(OMEGA_MIN <= self.u[1, k])
            self.solver.subject_to(self.u[1, k] <= OMEGA_MAX)

        # Define simulation variables
        self.usim = np.zeros((DU, 1))
        self.xsim = np.zeros((DX, 1))
        #To remove 
        self.xsim[:, 0] = np.array([0.3, 0.2, 0])
        self.usim[:, 0]  = np.array([0, 0])

        # Solver options 
        options = {"ipopt.print_level": 0, "print_time": 0}
        self.solver.solver('ipopt', options)

        # Create graph
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, NSIM)  # Limite initiale de l'axe x
        self.ax.set_ylim(-2, 2)  # Limite initiale de l'axe y
        self.ax.set_title("Output y")
        self.ax.grid()
        self.ax.legend(['y1', 'y2']) 
        self.x_data = []
        self.y1_data = []
        self.y2_data = []
        #plt.ion()   

    # Get position
    def position_callback(self, msg):
        self.last_pos_x = msg.pose.pose.position.x
        self.last_pos_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.last_yaw = yaw

    # Mpc controler
    def timer_callback(self):
        if self.last_pos_x != None : 
            if self.iteration < NSIM:

                # Define the objective function
                objective = 0
                for k in range(0, NPRED):
                    if k == 0 :
                        objective += cas.mtimes([(self.x[:, k] - XREF[:, self.iteration + k]).T, Q, (self.x[:, k] - XREF[:, self.iteration + k])]) \
                                 + cas.mtimes([(self.u[:, k] - self.usim[:, 0]).T, R, (self.u[:, k] -self.usim[:, 0])])
                    else : 
                        objective += cas.mtimes([(self.x[:, k] - XREF[:, self.iteration + k]).T, Q, (self.x[:, k] - XREF[:, self.iteration + k])]) \
                                 + cas.mtimes([(self.u[:, k] - self.u[:, k-1]).T, R, (self.u[:, k] -self.u[:, k-1])])
                #  Terminal cost
                objective += cas.mtimes([(self.x[:, NPRED] - XREF[:, self.iteration + NPRED]).T, P, (self.x[:, NPRED] - XREF[:, self.iteration + NPRED])])

                self.solver.minimize(objective)
                
                self.xsim[0, 0] = self.last_pos_x
                self.xsim[1, 0] = self.last_pos_y
                self.xsim[2, 0] = self.last_yaw

                self.solver.set_value(self.xinit, self.xsim[:, 0])

                sol = self.solver.solve()
                usol = sol.value(self.u)
                

                self.usim[:, 0] = usol[:, 0]
                #self.xsim[:, 0] = self.xsim[:, 0] + TE * np.array(f_dynamics(self.xsim[:, 0], self.usim[:, 0])).flatten()
                
                self.x_data.append(self.iteration)
                self.y1_data.append(self.xsim[0, 0])
                self.y2_data.append(self.xsim[1, 0])

                # self.ax.plot(self.x_data, self.y1_data, 'b')
                # self.ax.plot(self.x_data, self.y2_data, 'r')
                # if self.iteration == 0:
                #     plt.show() 
                #     print("Simulation started")
                # self.fig.canvas.draw() 
                # self.fig.canvas.flush_events()
                
                self.twist.linear.x = self.usim[0, 0]
                self.twist.angular.z = self.usim[1, 0]
                self.pub.publish(self.twist)
                self.iteration += 1
                # print(f"Position : x = {self.last_pos_x} y = {self.last_pos_y}")
            
            if self.iteration == NSIM:
                #plt.ioff()
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
                self.ax.plot(self.y1_data[:NSIM], label='x (sim)')
                self.ax.plot(self.y2_data[:NSIM], label='y (sim)')
                self.ax.plot(XREF[0, :NSIM], label='x (ref)', linestyle='--')
                self.ax.plot(XREF[1, :NSIM], label='y (ref)', linestyle='--')
                plt.show()
                print("Simulation ended")
                self.iteration += 1

def main(args=None):
    rclpy.init(args=args)
    node = ros_ref_tracking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        #node.get_logger().info('Keyboard Interrupt (SIGINT)')
    #finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

