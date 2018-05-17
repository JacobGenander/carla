function [c,ceq] = nonlinear_con(x, s0, additional_ineq_constraints_handle)
    

    u = x(:,1:2);
    s = x(:,3:6);

    % Create inequality constraints
    if nargin < 3
        c = [];
    else
        c = additional_ineq_constraints_handle(x);
    end

    % Create equality constraints
    T = size(x,1);
    ceq = zeros(T,4);
    for i = 2:T
        ceq(i-1,:) = step(s(i-1,:), u(i-1,:)) - s(i,:);
    end
    ceq(end+1,:) = x(1,3:end)-s0;
    
    ceq = ceq(:);    
end