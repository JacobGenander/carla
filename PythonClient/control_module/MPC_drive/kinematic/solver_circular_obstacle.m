clear
clc

LB_U = [-deg2rad(45), -1];
UB_U = [deg2rad(45), 1];
LB_S = [-inf -inf -inf -inf];
UB_S = [inf inf inf inf];

CREATE_NEW_TRAJ = true;
USE_LOSS_GRADIENT = true;
USE_CIRC_CONSTRAINT = true;

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
loss_func = @(z)loss(z,traj);

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
if USE_CIRC_CONSTRAINT
    n_cons = @(z)nonlinear_con(z, s0(1,:), @(w)-circ_constraint(w(:,3),w(:,4))); 
else
    n_cons = @(z)nonlinear_con(z, s0(1,:));
end

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
if USE_CIRC_CONSTRAINT
    subplot(2,3,[1,4])
    fimplicit(circ_constraint);
    axis([0 10 0 10]);
    hold on;
    plot_result(x, traj, [fval, output.constrviolation], lb, ub, circ_constraint)
else
    plot_result(x, traj, [fval, output.constrviolation], lb, ub)
end


