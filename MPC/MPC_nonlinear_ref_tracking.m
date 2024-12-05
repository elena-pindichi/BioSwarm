clc; close all; clear;

% Load CasADi
import casadi.*;

%% Parameters
N_pred_val = 25;     % Prediction horizon
Q_val = 1000;         % State weight value
R_val = 1;           % Input weight value
P_val = 1000;         % Terminal state value

% Define prediction and simulation steps
Te = 0.1;                           % Sampling time
Npred = N_pred_val;                 % Prediction horizon
Nsim = 300;                         % Number of simulation steps

% Physical parameters
r = 33e-3;           % Radius of wheels
L = 160e-3;          % Distance between wheels

% Define system dimensions
dx = 3;              % State dimensions: x, y, theta
du = 2;              % Control dimensions: V, omega

% Initial condition
x0 = [0.3; 0.2; 0];  % Initial state [x; y; theta]
% xref = [1; 5; 0];    % Reference state [x_ref; y_ref; theta_ref]
t = 0 : 2 * pi / (Nsim + Npred) : 2 * pi;
xref = [sin(t); cos(t); zeros(1, length(t))];
u0 = zeros(du, 1);

% Constraints on wheel angular velocities
Vmin = -10; Vmax = 10;           % Velocity limits
omegamin = -10; omegamax = 10;   % Angular velocity limits
xmin = -1 * 0.2 * ones(3, 1);    % State limits
xmax =  1 * 0.2 * ones(3, 1); 

% Weights for cost function
Q = Q_val * eye(dx);
R = R_val * eye(du);
P = P_val * Q;

%% CasADi nonlinear optimization
solver = casadi.Opti();

% Define optimization variables
x = solver.variable(dx, Npred + 1); % States over prediction horizon
u = solver.variable(du, Npred);     % Control inputs over prediction horizon (omega_r, omega_l)

% Define initial state as a parameter
xinit = solver.parameter(dx, 1);

% Nonlinear dynamics of the unicycle model with omega_r and omega_l
f_dynamics = @(x, u) [ u(1) * cos(x(3));
                       u(1) * sin(x(3));
                       u(2)];

% Add initial state constraint
solver.subject_to(x(:, 1) == xinit);

% Add constraints and dynamics for each prediction step
for k = 1 : Npred
    % State update constraint using discretized nonlinear dynamics
    solver.subject_to(x(:, k+1) == x(:, k) + Te * f_dynamics(x(:, k), u(:, k)));
    solver.subject_to(xmin <= x(:, k + 1) - x(:, k) <= xmax)
    
    % Control input constraints
    solver.subject_to(Vmin <= u(1, k) <= Vmax);
    solver.subject_to(omegamin <= u(2, k) <= omegamax);
end

%% Define the objective function
% Define the solver
options.ipopt.print_level = 0;
options.ipopt.sb= 'yes';
options.print_time = 0;

% simulation loop
usim = zeros(du, Nsim);
xsim = zeros(dx, Nsim);
xsim(:, 1) = x0;
usim_init = u0;

for i = 1 : Nsim
    objective = 0;
    for k = 1 : Npred
        if k ~= 1
            objective = objective + (x(:, k) - xref(:, i + k - 1))' * Q * (x(:, k) - xref(:, i + k - 1)) ...
                                                                        + (u(:, k) - u(:, k - 1))' * ...
                                                                        R * (u(:, k) - u(:, k - 1));
        else
            objective = objective + (x(:, k) - xref(:, i + k - 1))' * Q * (x(:, k) - xref(:, i + k - 1)) ...
                                                                        + u(:, k)' * R * u(:, k);
        end
    end

    objective = objective + (x(:, Npred + 1) - xref(:, i + Npred))' * P * (x(:, Npred + 1) ...
                                                                        - xref(:, i + Npred));
    solver.minimize(objective)
    solver.solver('ipopt', options)
    solver.set_value(xinit, xsim(:, i))
    sol = solver.solve();
    usol = sol.value(u);
    usim(:, i) = usol(:, 1);
    xsim(:, i + 1) = xsim(:, i) + Te * f_dynamics(xsim(:, i), usim(:, i));

end

%% Plot results
figure
stem(xsim(1, :));
hold on
stem(xsim(2, :));
hold on
plot(xref(1, 1:Nsim))
hold on
plot(xref(2, 1:Nsim))
grid

% figure;
% subplot(3,1,1);
% plot(xsim(1,:), 'LineWidth', 1.5); hold on;
% plot(xref(1) * ones(1, Nsim+1), '--r');
% ylabel('x position');
% legend('x', 'x_{ref}');
% grid
% 
% subplot(3,1,2);
% plot(xsim(2,:), 'LineWidth', 1.5); hold on;
% plot(xref(2) * ones(1, Nsim+1), '--r');
% ylabel('y position');
% legend('y', 'y_{ref}');
% grid
% 
% subplot(3,1,3);
% plot(xsim(3,:), 'LineWidth', 1.5); hold on;
% plot(xref(3) * ones(1, Nsim+1), '--r');
% ylabel('\theta (orientation)');
% legend('\theta', '\theta_{ref}');
% grid
% 
% figure;
% subplot(2,1,1);
% stairs(usim(1,:), 'LineWidth', 1.5);
% ylabel('Angular Velocity \omega_r');
% legend('\omega_r');
% grid
% 
% subplot(2,1,2);
% stairs(usim(2,:), 'LineWidth', 1.5);
% ylabel('Angular Velocity \omega_l');
% legend('\omega_l');
% grid
% 
% figure
% scatter(xsim(1, :), xsim(2, :))
% grid
% title('State space');
% xlabel 'x1'
% ylabel 'x2'
