clear
clc

LB_U = [-deg2rad(45), -200];
UB_U = [deg2rad(45), 200];
LB_S = [-inf -inf -inf -inf];
UB_S = [inf inf inf inf];

CREATE_NEW_TRAJ = true;
USE_LOSS_GRADIENT = true;

% Load the distance map to sixth object
dMap = csvread('../data/distance_maps/distance_object_6.csv');

% Create trajectory to be followed
if CREATE_NEW_TRAJ
    f = figure();
    contour(dMap, [0 0], 'k')
    grid minor
    traj = get_trajectory(f, [0 2600 0 2200]);
    save ('traj.mat', 'traj')
else
    load('traj.mat', 'traj')
end
T = size(traj,1);

% Create loss function
loss_func = @(z)loss(z,traj);

% Create inequality coinstraint handle
object_ineq_constraint = @(z)-interpolate(dMap, z(:,3), z(:,4));

% Choose initial point (feasible)
u0 = zeros(T,2);

delta_y = traj(2,2)-traj(1,2);
delta_x = traj(2,1)-traj(1,1);
psi0 = atan2(delta_y, delta_x);
v0 = norm([delta_y delta_x]);

s0 = [traj(1,:), psi0, v0];
for i = 2:T
    s0(i,:) = kinematic_model(s0(i-1,:), u0(i-1,:));
end
z0 = [u0, s0];

% There are no linear constraints
A = [];
b = [];
Aeq = [];
beq = [];

% Non-linear constraints
n_cons = @(z)nonlinear_con(z, s0(1,:), object_ineq_constraint); 

% Lower and upper bound for Z
lb = [repmat(LB_U, T, 1), repmat(LB_S, T, 1)];
ub = [repmat(UB_U, T, 1), repmat(UB_S, T, 1)];

% Set optimization options
options = optimoptions('fmincon','Display','iter', ...
                       'MaxFunctionEvaluations', 1e5, ...
                       'Algorithm', 'interior-point', ...
                       'SpecifyObjectiveGradient',USE_LOSS_GRADIENT, ...
                       'CheckGradient', USE_LOSS_GRADIENT, ...
                       'FiniteDifferenceType', 'central', ...
                       'MaxIterations', 1e4, ...
                       'MaxFunctionEvaluations', 1e6);

% Perform optimization
[x, fval, ~, output] = fmincon(loss_func, z0, A, b, Aeq, beq, lb, ub, n_cons, options);

% Plot the results
subplot(2,3,[1,4]);
contour(dMap, [0 0], 'k');
hold on;
plot_result(x, traj, [fval, output.constrviolation], lb, ub)

