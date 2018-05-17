function [u_optimal, z_optimal, fval, output] = CARLA_MPC(x, ref)
%MPC(x,ref) Computes the optimal input sequence in order to follow ref
%   This function computes the optimal input sequence in order to follow as
%   close as possible the xy trajectory specified in the argument ref.
%
%--------------------------------------------------------------------------
% Inputs
%--------------------------------------------------------------------------
%
% - x: The current state vector, 1x4, comprised of [x position, y position,
%   heading angle, velocity], in this order.
%
% - ref: The desired reference points in the xy plane, Tx2 vector, where
%   each row is made of an x y pair. T is the number of timesteps for the
%   reference.
%
%
%--------------------------------------------------------------------------
% Outputs
%--------------------------------------------------------------------------
%
% - u_optimal: The optimal input sequence computed by the MPC. Tx3 vector,
%   where the first column is the steering wheel angle (in radians), the 
%   second column is the throttle, and the third is the brake.
%
% - z_optimal: Matrix comprised of [u, s]. u is [delta_f, a], where delta_f
%   is the steering wheel angle in radians, and a is the acceleration of
%   the pivot point of the car. s is the predicted states, a Tx4 matrix 
%   where each row is comprised of [x, y, heading angle, velocity].
%
% - fval: MSE of the predicted state sequence w.r.t. the reference xy
%   points, for debugging purposes.
%
% - output: Complete output of the optimization algorithm used in this
%   script (fmincon), for debugging purposes.
%

    persistent object_ineq_constraint dMap2 dMap3 dMap4 dMap5 dMap6 ... 
        dMap7 dMap8 dMap9 net;
    
    if isempty(object_ineq_constraint)
    
        % Load the distance map to all objects
        load ../data/distance_maps/dMaps dMap2 dMap3 dMap4 dMap5 dMap6 ...
            dMap7 dMap8 dMap9
        
        % Create inequality constraint handle
        object_ineq_constraint = @(z)[-interpolate(dMap2, z(:,3), z(:,4)); ...
                              -interpolate(dMap3, z(:,3), z(:,4)); ...
                              -interpolate(dMap4, z(:,3), z(:,4)); ...
                              -interpolate(dMap5, z(:,3), z(:,4)); ...
                              -interpolate(dMap6, z(:,3), z(:,4)); ...
                              -interpolate(dMap7, z(:,3), z(:,4)); ...
                              -interpolate(dMap8, z(:,3), z(:,4)); ...
                              -interpolate(dMap9, z(:,3), z(:,4))];  
                          
        % Load the neural network that maps desired acceleration to 
        % throttle and brake
        load ../data/throttle_measurements/net.mat net
                                 
    end
                    
    
    T = size(ref,1);   
    
    % Set optimization options
    options = optimoptions('fmincon','Display','iter', ...
                           'MaxFunctionEvaluations', 1e5, ...
                           'Algorithm', 'interior-point', ...
                           'SpecifyObjectiveGradient',false, ...
                           'CheckGradient', false, ...
                           'FiniteDifferenceType', 'central', ...
                           'MaxIterations', 1e4, ...
                           'MaxFunctionEvaluations', 1e6); 
                       
    % Upper and lower bounds for Z
    LB_U = [deg2rad(-70), -100];
    UB_U = [deg2rad(70), 200];
    LB_S = [-inf -inf -inf -inf];
    UB_S = [inf inf inf inf];
    lb = [repmat(LB_U, T, 1), repmat(LB_S, T, 1)];
    ub = [repmat(UB_U, T, 1), repmat(UB_S, T, 1)];
    
    % Choose feasible initial point
    u0 = zeros(T,2);
    psi0 = x(3);
    v0 = x(4);

    s0 = [ref(1,:), psi0, v0];
    for i = 2:T
        s0(i,:) = kinematic_model(s0(i-1,:), u0(i-1,:));
    end
    z0 = [u0, s0];
    
    % Create loss function handle
    loss_func = @(z)loss(z, ref);
    
    % Non-linear constraints
    n_cons = @(z)nonlinear_con(z, s0(1,:), object_ineq_constraint); 
    
    % There are no linear constraints
    A = [];
    b = [];
    Aeq = [];
    beq = [];
        
    % Perform optimization
    [z_optimal, fval, ~, output] = fmincon(loss_func, z0, A, b, Aeq, beq, lb, ub, n_cons, options);
    
    % Compute throttle and brake with neural network
    input = struct('a_v', z_optimal(:,2), ...
                   'delta_f', z_optimal(:,1), ...
                   'forward_speed', z_optimal(:,6));
    [throttle, brake] = evaluate_net(net, input);
    
    % Construct optimal input vector
    u_optimal = [z_optimal(:,1), throttle', brake'];


end

