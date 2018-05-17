function [L, g] = loss(z, reference)
%LOSS Summary of this function goes here
%   Detailed explanation goes here
    
    position = z(:,3:4);
    L = sum((position(:)-reference(:)).^2);
    
    if nargout > 1 % Compute the gradient
        T = size(z,1);
        g_xy = 2*(position-reference);
        g = [zeros(T,2), g_xy , zeros(T,2)];
    end

end

