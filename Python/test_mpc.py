import numpy as np
import casadi as cas
import time
import matplotlib.pyplot as plt

# The double integrator dynamics System data

# State-space model: x+ = Ax + Bu; y = Cx + Du
h = 0.5

A = np.block([[np.eye(2), h*np.eye(2)],
              [np.zeros((2,2)), np.eye(2)]])
B = np.vstack([np.zeros((2,2)), h*np.eye(2)])
C = np.hstack([np.eye(2), np.zeros((2,2))])
D = np.zeros((2,2))

# Model dimension
dx, du = np.shape(B)
dy = np.shape(C)[0]

# Initial conditions
x0 = np.array([0.0, 0.0, 0.0, 0.0])
u0 = np.zeros((du, 1))

# Goal 
xgoal = np.array([1.0, 2.0, 0.0, 0.0])

# Constraints
umin = -1 * 0.25
umax = +1 * 0.25
delta_u_min = -0.1
delta_u_max = +0.1
ymin = -10
ymax = +10
#xmin = np.array([ymin, -0.2])
#xmax = np.array([ymax, 0.5])

# Define control parameters
# Weighting matrices 
Q = np.eye(dx) # cost for the state x
R = 1  # cost for the input u
Qy = np.eye(dy)  # cost for the output y
P = 10 * Q  # cost for the terminal cost
# Number of predictions and simulations
Npred = 5
Nsim = 100

# Optimization problem using CasADi
solver = cas.Opti()  # create an Opti object

# Define variables
x = solver.variable(dx, Npred + 1)
u = solver.variable(du, Npred)
xinit = solver.parameter(dx, 1)
uinit = solver.parameter(du, 1)

# Initialize constraints
solver.subject_to(x[:, 0] == xinit)

for k in range(0, Npred):
    solver.subject_to(x[:, k + 1] == cas.mtimes(A, x[:, k]) + cas.mtimes(B, u[:, k]))  # dynamics

    # input magnitude constraints
    solver.subject_to(umin <= u[:, k])
    solver.subject_to(u[:, k] <= umax)

    # state magnitude constraints
    solver.subject_to(ymin <= cas.mtimes(C, x[:, k]) + cas.mtimes(D, u[:, k]))
    solver.subject_to(cas.mtimes(C, x[:, k]) + cas.mtimes(D, u[:, k]) <= ymax)
    
    # additional state constraints solver.subject_to(xmin[1] <= x[1, k] <= xmax[1])  

    if k == 0:
        solver.subject_to(delta_u_min <= u[:, k] - uinit)
        solver.subject_to(u[:, k] - uinit <= delta_u_max)
    else:
        solver.subject_to(delta_u_min <= u[:, k] - u[:, k - 1])
        solver.subject_to(u[:, k] - u[:, k - 1] <= delta_u_max)

# Initialize objective
objective = 0

for k in range(0, Npred):
    if k == 0:
        objective += cas.mtimes(cas.mtimes(cas.transpose(x[:, k] - xgoal), Q), x[:, k] - xgoal) + cas.mtimes(cas.mtimes(cas.transpose(u[:, k] - u0), R), u[:, k] - u0)  # quadratic cost function
    else:
        objective += cas.mtimes(cas.mtimes(cas.transpose(x[:, k] - xgoal), Q), x[:, k] - xgoal) + cas.mtimes(cas.mtimes(cas.transpose(u[:, k] - u[:, k-1]), R), u[:, k] - u[:, k-1])  # quadratic cost function

objective += cas.mtimes(cas.mtimes(cas.transpose(x[:, Npred] - xgoal), P), x[:, Npred] - xgoal)

# Define the solver
solver.minimize(objective)
options = {'ipopt' : {'print_level': 0, 'sb': 'yes'}, 'print_time': 0}
solver.solver('ipopt', options)

# Création de la figure et des axes
fig, ax = plt.subplots()
ax.set_xlim(0, Nsim)  # Limite initiale de l'axe x
ax.set_ylim(-1, 3)  # Limite initiale de l'axe y
ax.set_title("Output y")
ax.grid()
ax.legend(['y1', 'y2'], loc='upper right')
x_data = []
y1_data = []
y2_data = []
plt.ion() 

# Simulation loop
usim = np.zeros((du, Nsim))
ysim = np.zeros((dy, Nsim))
xsim = np.zeros((dx, Nsim + 1))
xsim[:, 0] = x0
usim_init = u0

# time_start = time.time()
for i in range(Nsim):
    solver.set_value(xinit, xsim[:, i])
    solver.set_value(uinit, usim_init)

    sol = solver.solve()
    usol = sol.value(u)
    
    usim_init = usol[:,0]

    usim[:, i] = usol[:, 0]
    xsim[:, i + 1] = A @ xsim[:, i] + B @ usim[:, i]  # update the dynamics
    ysim[:, i] = C @ xsim[:, i] + D @ usim[:, i]  # update the dynamics
    
    x_data.append(i)
    y1_data.append(ysim[0,i])
    y2_data.append(ysim[1,i])

    if i > 0 :
        stemlines1.remove()  # Supprime la version précédente pour éviter l'empilement
        stemlines2.remove()
    
    stemlines1 = ax.stem(x_data, y1_data, linefmt='b-', markerfmt='bo', basefmt=" ")
    stemlines2 = ax.stem(x_data, y2_data, linefmt='r-', markerfmt='ro', basefmt=" ")
    if i == 0:
        plt.show() 
    fig.canvas.draw() 
    fig.canvas.flush_events()
    time.sleep(0.01)

plt.ioff()  # Désactiver le mode interactif une fois terminé



# timer_end = time.time()
# time_elapsed = timer_end - time_start
# print(f'Total time: {time_elapsed} (s)')
# Plot the results


# plt.figure(1)
# plt.stem(ysim[0, :], label='y1')
# plt.stem(ysim[1, :], label='y2', linefmt='r-', markerfmt='ro')
# plt.title('Output y')
# plt.grid()
# plt.legend()
# #plt.show()


# plt.figure(2)
# plt.scatter(xsim[0, :], xsim[1, :], edgecolors='red', facecolors='none')
# plt.title('State space')
# plt.grid()
# plt.xlabel('x1')
# plt.ylabel('x2')
# #plt.show()

# plt.figure(3)
# plt.stem(usim[0, :], label='u1')
# plt.stem(usim[1, :], label='u2', linefmt='r-', markerfmt='ro')
# plt.title('Input u')
# plt.grid()
# plt.legend()
# plt.show()

# # Analyze the results
# error = np.sqrt(np.sum(xsim**2, axis=0))
# Avg_error = np.mean(error)
# print(f'Average error: {Avg_error}')

# Avg_u = np.mean(np.abs(usim), axis=1)
# print(f'Average input: {Avg_u}')