clc; close all; clear;

% Load CasADi
import casadi.*;

%% Define obstacle
A_ob = [1 1; 1 -1; -1 -1; -1 1];
b_ob = [1; 1; 1; 1];
O = Polyhedron(A_ob, b_ob);
H = O.A;
b = O.b;
M = 10^5;

%% Parameters
N_pred_val = 25;     % Prediction horizon
Q_val = 100;         % State weight value
R_val = 1;           % Input weight value
P_val = 200;         % Terminal state value

% Physical parameters
r = 33e-3;           % Radius of wheels
L = 160e-3;          % Distance between wheels

% Define system dimensions
dx = 3;              % State dimensions: x, y, theta
du = 2;              % Control dimensions: omega_r (right wheel), omega_l (left wheel)

% Initial condition
x0 = [0.3; 0.2; 0];  % Initial state [x; y; theta]
xref = [1; 5; 0];    % Reference state [x_ref; y_ref; theta_ref]

% Constraints on wheel angular velocities
omega_r_min = -10; omega_r_max = 10;   % Right wheel angular velocity limits
omega_l_min = -10; omega_l_max = 10;   % Left wheel angular velocity limits
xmin = -1 * 0.1 * ones(3, 1);          % State limits
xmax =  1 * 0.1 * ones(3, 1); 

% Weights for cost function
Q = Q_val * eye(dx);
R = R_val * eye(du);
P = P_val * Q;

% Define prediction and simulation steps
Te = 0.1;                           % Sampling time
Npred = N_pred_val;                 % Prediction horizon
Nsim = 300;                         % Number of simulation steps

%% CasADi nonlinear optimization
solver = casadi.Opti();

% Define optimization variables
x = solver.variable(dx, Npred + 1); % States over prediction horizon
u = solver.variable(du, Npred);     % Control inputs over prediction horizon (omega_r, omega_l)

% Define initial state as a parameter
xinit = solver.parameter(dx, 1);

% Nonlinear dynamics of the unicycle model with omega_r and omega_l
f_dynamics = @(x, u) [ (r/2) * (u(1) + u(2)) * cos(x(3));
                       (r/2) * (u(1) + u(2)) * sin(x(3));
                       (r/L) * (u(1) - u(2)) ];

% Add initial state constraint
solver.subject_to(x(:, 1) == xinit);

% Add constraints and dynamics for each prediction step
for k = 1 : Npred
    % State update constraint using discretized nonlinear dynamics
    solver.subject_to(x(:, k+1) == x(:, k) + Te * f_dynamics(x(:, k), u(:, k)));
    solver.subject_to(xmin <= x(:, k + 1) - x(:, k) <= xmax)
    
    % Control input constraints
    solver.subject_to(omega_r_min <= u(1, k) <= omega_r_max);
    solver.subject_to(omega_l_min <= u(2, k) <= omega_l_max);
end

%% Define the objective function
objective = 0;
for k = 1 : Npred
    % Stage cost (penalty for deviation from reference)
    objective = objective + (x(:, k) - xref)' * Q * (x(:, k) - xref) + u(:, k)' * R * u(:, k);
end
% Terminal cost
objective = objective + (x(:, Npred + 1) - xref)' * P * (x(:, Npred + 1) - xref);
solver.minimize(objective);

% Solver options
options.ipopt.print_level = 0;
options.ipopt.sb = 'yes';
options.ipopt.tol = 1e-6;
options.print_time = 0;
solver.solver('ipopt', options);

% Simulation loop
xsim = zeros(dx, Nsim + 1);
xsim(:, 1) = x0;
usim = zeros(du, Nsim);

%% Initialize CasADi variables for state and input
solver.set_value(xinit, x0);

for i = 1 : Nsim
    % Solve the nonlinear MPC optimization
    sol = solver.solve();
    
    % Extract optimal control inputs
    usol = sol.value(u);
    
    % Apply first control input
    usim(:, i) = usol(:, 1);
    
    % Simulate one step using dynamics
    xsim(:, i+1) = xsim(:, i) + Te * f_dynamics(xsim(:, i), usim(:, i));
    
    % Update initial state parameter in CasADi for the next iteration
    solver.set_value(xinit, xsim(:, i+1));
end

%% Plot results
figure;
subplot(3,1,1);
plot(xsim(1,:), 'LineWidth', 1.5); hold on;
plot(xref(1) * ones(1, Nsim+1), '--r');
ylabel('x position');
legend('x', 'x_{ref}');
grid

subplot(3,1,2);
plot(xsim(2,:), 'LineWidth', 1.5); hold on;
plot(xref(2) * ones(1, Nsim+1), '--r');
ylabel('y position');
legend('y', 'y_{ref}');
grid

subplot(3,1,3);
plot(xsim(3,:), 'LineWidth', 1.5); hold on;
plot(xref(3) * ones(1, Nsim+1), '--r');
ylabel('\theta (orientation)');
legend('\theta', '\theta_{ref}');
grid

figure;
subplot(2,1,1);
stairs(usim(1,:), 'LineWidth', 1.5);
ylabel('Angular Velocity \omega_r');
legend('\omega_r');
grid

subplot(2,1,2);
stairs(usim(2,:), 'LineWidth', 1.5);
ylabel('Angular Velocity \omega_l');
legend('\omega_l');
grid

figure
scatter(xsim(1, :), xsim(2, :))
grid
title('State space');
xlabel 'x1'
ylabel 'x2'
