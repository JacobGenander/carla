function [c,ceq] = nonlinear_con(z, s0, additional_ineq_constraints_handle)
% NONLINEAR_CON() computes the nonlinear constraints for the MPC problem.
%
% This function returns the nonlinear equality and inequality functions of
% the MPC problem computed at the provided Z point.
%
%--------------------------------------------------------------------------
% Inputs
%--------------------------------------------------------------------------
%
% - z: The vector being optimized by MPC (Tx6, where T is the prediction
%   horizon). Comprised of [U, X], where U is the input vector and X is the
%   state vector (both are described in the documentation of the kinematic
%   model being used in the optimization.
%
% - s0: Initial value for X (1x4). Used for guaranteeing that the initial
%   predicted state matches the measured one.
%
% - additional_ineq_constraints_handle: Handle to a function that computes
%   other inequality constraints than the used in this function. This
%   handle function should take as argument the z vector ([U,X]), and
%   output a vector (Nx1) with the value of the inequality constraints
%   computed at the point z.
%
%
%--------------------------------------------------------------------------
% Outputs
%--------------------------------------------------------------------------
%
% - c: output from additional_ineq_constraints. Empty if
%   additional_ineq_constraints is not specified.
%
% - ceq: (T*4x1) vector with the constraints values. The first 4 elements
%   are the constraints for the first time-step, the second 4 for the 
%   second time-step, and so on.
%
%

    u = z(:,1:2);
    s = z(:,3:6);

    % Create inequality constraints
    if nargin < 3
        c = [];
    else
        c = additional_ineq_constraints_handle(z);
    end

    % Create equality constraints
    T = size(z,1);
    ceq = zeros(T,4);
    for i = 2:T
        ceq(i-1,:) = kinematic_model(s(i-1,:), u(i-1,:)) - s(i,:);
    end
    ceq(end+1,:) = z(1,3:end)-s0;
    
    ceq = ceq(:);    
end