function [L, g] = loss(z, reference)
%LOSS() computes the loss function evaluated at the provided Z point.
%
% This function is used by solver_carlaobstacles as the optimization
% objective.
    
    position = z(:,3:4);
    L = 0.99*sum((position(:)-reference(:)).^2);
    L = L + 0.01*sum(z(:,1).^2); % penalize first input magnitude

end

