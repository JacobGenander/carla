clear
clc

LB_U = [-deg2rad(3), -20];
UB_U = [deg2rad(3), 20];
LB_S = [-inf -inf -inf -inf];
UB_S = [inf inf inf inf];

CREATE_NEW_TRAJ = true;
USE_LOSS_GRADIENT = true;

MAP_WINDOW = [550 800 300 550];

% Load the distance map to all objects (.csv files inside csv/ folder)
load ../data/distance_maps/dMaps

% Create trajectory to be followed
if CREATE_NEW_TRAJ
    f = figure();
    
    % Plot all the obstacles
    [x_temp,y_temp]=find(dMap2==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    hold on;
    [x_temp,y_temp]=find(dMap3==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap4==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap5==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap6==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap7==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap8==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    [x_temp,y_temp]=find(dMap9==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
    axis(MAP_WINDOW)
    grid minor
    
    % Get reference points with mouse clicks
    traj = get_trajectory(f, MAP_WINDOW);
    save ('traj.mat', 'traj');
else
    load('traj.mat', 'traj');
end
T = size(traj,1);

% Create loss function
loss_func = @(z)loss(z,traj);

% Create inequality coinstraint handle
object_ineq_constraint = @(z)[-interpolate(dMap2, z(:,3), z(:,4)); ...
                              -interpolate(dMap3, z(:,3), z(:,4)); ...
                              -interpolate(dMap4, z(:,3), z(:,4)); ...
                              -interpolate(dMap5, z(:,3), z(:,4)); ...
                              -interpolate(dMap6, z(:,3), z(:,4)); ...
                              -interpolate(dMap7, z(:,3), z(:,4)); ...
                              -interpolate(dMap8, z(:,3), z(:,4)); ...
                              -interpolate(dMap9, z(:,3), z(:,4))];

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
[x_temp,y_temp]=find(dMap2==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
hold on;
[x_temp,y_temp]=find(dMap3==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap4==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap5==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap6==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap7==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap8==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
[x_temp,y_temp]=find(dMap9==0); scatter(y_temp, x_temp, 4, 'Filled', 'k')
axis(MAP_WINDOW)


plot_result(x, traj, [fval, output.constrviolation], lb, ub)

