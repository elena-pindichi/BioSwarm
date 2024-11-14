import numpy as np
import casadi as cas
import time
import matplotlib.pyplot as plt

# Constants
#  Parameters
NPRED = 25     # Prediction horizon
NSIM = 500
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
X0 = np.array([0.3, 0.2, 0])  # Initial state [x; y; theta]
XREF = np.array([5, 5, 0])    # Reference state [x_ref; y_ref; theta_ref]

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

# Optimization problem using CasADi
solver = cas.Opti()  # create an Opti object

#  Define optimization variables
x = solver.variable(DX, NPRED + 1)  # States over prediction horizon
u = solver.variable(DU, NPRED)      # Control inputs over prediction horizon (omega_r, omega_l)
xinit = solver.parameter(DX, 1)

#  Nonlinear dynamics of the unicycle model with omega_r and omega_l
f_dynamics = cas.vertcat(
    (RADIUS / 2) * (u[0] + u[1]) * cas.cos(x[2]),
    (RADIUS / 2) * (u[0] + u[1]) * cas.sin(x[2]),
    (RADIUS / L) * (u[0] - u[1])
)
f_dynamics_func = cas.Function('f_dynamics', [x, u], [f_dynamics]) # Wrap it as a CasADi function


# Initialize constraints
solver.subject_to(x[:, 0] == xinit)
for k in range(0, NPRED) : 
    # State update constraint using discretized nonlinear dynamics
    solver.subject_to(x[:, k+1] == (x[:, k] + TE * f_dynamics_func(x[:, k], u[:, k])))
    solver.subject_to(XMIN <= (x[:, k + 1] - x[:, k]))
    solver.subject_to((x[:, k + 1] - x[:, k]) <= XMAX)
    # % solver.subject_to(-H * x(1:2, k+1) <= -b + M * alpha(:, k+1))
    # % solver.subject_to(sum(alpha(:, k+1)) <= 3)
    
    #  Control input constraints
    solver.subject_to(OMEGA_R_MIN <= u[0, k])
    solver.subject_to(u[0, k] <= OMEGA_R_MAX)
    solver.subject_to(OMEGA_L_MIN <= u[1, k])
    solver.subject_to(u[1, k] <= OMEGA_L_MAX)

# Define the objective function
objective = 0
for k in range(0, NPRED):
    #  Stage cost (penalty for deviation from reference)
    # objective += cas.times(cas.times(cas.transpose(x[:, k] - XREF), Q), x[:, k] - XREF) + cas.times(cas.times(cas.transpose(u[:, k]), R), u[:, k])
    objective += cas.mtimes([cas.transpose(x[:, k] - XREF), Q, x[:, k] - XREF]) + cas.mtimes([cas.transpose(u[:, k]), R, u[:, k]])
#  Terminal cost
objective += cas.mtimes([cas.transpose(x[:, NPRED] - XREF), Q, x[:, NPRED] - XREF])
# objective += cas.times(cas.times(cas.transpose(x[:, NPRED] - XREF), P), x[:, NPRED] - XREF)
solver.minimize(objective)

# Solver options 
options = {'ipopt' : {'print_level': 0, 'sb': 'yes', 'tol' : 1e-6}, 'print_time': 0}
solver.solver('ipopt', options)

# Simulation loop
usim = np.zeros((DU, NSIM))
xsim = np.zeros((DX, NSIM + 1))
xsim[:, 0] = X0

# Initialize CasADi variables for state and input
solver.set_value(xinit, xsim[:, 0])
for i in range(0, NSIM):
    # Solve the nonlinear MPC optimization
    sol = solver.solve()
    usol = sol.value(u)
    # Apply first control input
    usim[:, i] = usol[:, 0]
    # Simulate one step using dynamics
    xsim[:, i+1] = xsim[:, i] + TE * f_dynamics_func(xsim[:, i], usim[:, i]).full().flatten()
    # Update initial state parameter in CasADi for the next iteration
    solver.set_value(xinit, xsim[:, i+1])


# Plot x, y, theta positions with references
plt.figure(figsize=(8, 10))

# x position
plt.subplot(3, 1, 1)
plt.plot(xsim[0, :], label='x', linewidth=1.5)
plt.plot(XREF[0] * np.ones(NSIM+1), '--r', label='x_ref')
plt.ylabel('x position')
plt.legend()
plt.grid()

# y position
plt.subplot(3, 1, 2)
plt.plot(xsim[1, :], label='y', linewidth=1.5)
plt.plot(XREF[1] * np.ones(NSIM+1), '--r', label='y_ref')
plt.ylabel('y position')
plt.legend()
plt.grid()

# Theta (orientation)
plt.subplot(3, 1, 3)
plt.plot(xsim[2, :], label=r'$\theta$', linewidth=1.5)
plt.plot(XREF[2] * np.ones(NSIM+1), '--r', label=r'$\theta_{ref}$')
plt.ylabel(r'$\theta$ (orientation)')
plt.legend()
plt.grid()

plt.tight_layout()
# plt.show()

# Plot control inputs (angular velocities)
plt.figure(figsize=(8, 6))

# Right wheel angular velocity
plt.subplot(2, 1, 1)
plt.step(range(NSIM), usim[0, :], label=r'$\omega_r$', linewidth=1.5)
plt.ylabel(r'Angular Velocity $\omega_r$')
plt.legend()
plt.grid()

# Left wheel angular velocity
plt.subplot(2, 1, 2)
plt.step(range(NSIM), usim[1, :], label=r'$\omega_l$', linewidth=1.5)
plt.ylabel(r'Angular Velocity $\omega_l$')
plt.legend()
plt.grid()

plt.tight_layout()
# plt.show()

# Scatter plot of x1 vs x2 in state space
plt.figure(figsize=(6, 6))
plt.scatter(xsim[0, :], xsim[1, :], label='State space')
plt.grid()
plt.title('State space')
plt.xlabel('x1')
plt.ylabel('x2')
plt.legend()
plt.show()