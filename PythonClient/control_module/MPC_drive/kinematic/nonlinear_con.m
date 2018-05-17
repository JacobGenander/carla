function [c,ceq] = nonlinear_con(z, s0, additional_ineq_constraints_handle)
    

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