clear
clc

LB_U = [-deg2rad(20), -1];
UB_U = [deg2rad(20), 1];
LB_S = [-inf -inf -inf 0];
UB_S = [inf inf inf 5];

CREATE_NEW_TRAJ = false;
USE_LOSS_GRADIENT = true;
USE_CIRC_CONSTRAINT = false;

% Create circular constraint
if USE_CIRC_CONSTRAINT
    circ_pos = [5, 5];
    circ_R = 1;
    circ_constraint = @(x,y)(x-circ_pos(1)).^2 + ...
                          (y-circ_pos(2)).^2 - circ_R^2;
end

% Create trajectory to be followed
if CREATE_NEW_TRAJ
    f = figure();
    if USE_CIRC_CONSTRAINT
        fimplicit(circ_constraint);
    end
    traj = get_trajectory(f, [0 10 0 10]);
    save ('traj.mat', 'traj')
else
    load('traj.mat', 'traj')
end
T = size(traj,1);

% Create loss function
loss_func = @(s)loss(s,traj);

% Choose initial point (feasible)
u0 = zeros(T,2);

delta_y = traj(2,2)-traj(1,2);
delta_x = traj(2,1)-traj(1,1);
theta0 = atan2(delta_y, delta_x);
v0 = norm([delta_y delta_x]);

s0 = [traj(1,:), theta0, v0];
for i = 2:T
    s0(i,:) = step(s0(i-1,:), u0(i-1,:));
end
x0 = [u0, s0];

% There are no linear constraints
A = [];
b = [];
Aeq = [];
beq = [];

% Non-linear constraints
if USE_CIRC_CONSTRAINT
    n_cons = @(x)nonlinear_con(x, s0(1,:), @(z)-circ_constraint(z(:,3),z(:,4))); 
else
    n_cons = @(x)nonlinear_con(x, s0(1,:));
end

% Lower and upper bound for state and input
lb = [repmat(LB_U, T, 1), repmat(LB_S, T, 1)];
ub = [repmat(UB_U, T, 1), repmat(UB_S, T, 1)];

options = optimoptions('fmincon','Display','iter', ...
                       'MaxFunctionEvaluations', 1e5, ...
                       'Algorithm', 'interior-point', ...
                       'SpecifyObjectiveGradient',USE_LOSS_GRADIENT, ...
                       'CheckGradient', USE_LOSS_GRADIENT, ...
                       'FiniteDifferenceType', 'central', ...
                       'MaxIterations', 1e4, ...
                       'MaxFunctionEvaluations', 1e6);
[x, fval, ~, output] = fmincon(loss_func, x0, A, b, Aeq, beq, lb, ub, n_cons, options);

% Plot the results
if USE_CIRC_CONSTRAINT
    plot_result(x, traj, [fval, output.constrviolation], lb, ub, circ_constraint)
else
    plot_result(x, traj, [fval, output.constrviolation], lb, ub)
end


